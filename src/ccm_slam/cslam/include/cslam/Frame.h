/**
* This file is part of CCM-SLAM.
*
* Copyright (C): Patrik Schmuck <pschmuck at ethz dot ch> (ETH Zurich)
* For more information see <https://github.com/patriksc/CCM-SLAM>
*
* CCM-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* CCM-SLAM is based in the monocular version of ORB-SLAM2 by Raúl Mur-Artal.
* CCM-SLAM partially re-uses modules of ORB-SLAM2 in modified or unmodified condition.
* For more information see <https://github.com/raulmur/ORB_SLAM2>.
*
* CCM-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with CCM-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CSLAM_FRAME_H_
#define CSLAM_FRAME_H_

//C++
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Datatypes.h>
#include <cslam/ORBVocabulary.h>
#include <cslam/ORBextractor.h>
#include <cslam/MapPoint.h>
#include <cslam/KeyFrame.h>
#include <cslam/BundledKeyFrames.h>
#include <cslam/Converter.h>

//Thirdparty
#include <thirdparty/DBoW2/DBoW2/BowVector.h>
#include <thirdparty/DBoW2/DBoW2/FeatureVector.h>

using namespace estd;

namespace cslam{

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 75

struct X3D
{

    X3D(cv::Mat mPos_, int cameraId_, int keypointId_, const int mnScaleLevels_,  const float mfScaleFactor_, const float mfLogScaleFactor_, vector<cv::Mat> mvpDescriptors_):
            mPos(mPos_.clone()), cameraId(cameraId_), keypointId(keypointId_), mfMinDistance(0), mfMaxDistance(0), mnScaleLevels(mnScaleLevels_), mfScaleFactor(mfScaleFactor_), mfLogScaleFactor(mfLogScaleFactor_),
            mvpDescriptors(mvpDescriptors_)
    {

    }

    cv::Mat mPos;
    int cameraId;
    int keypointId;

    // Best descriptor to fast matching
    cv::Mat mDescriptor;

    vector<cv::Mat> mvpDescriptors;  //keypoints get from frame class

    float mTrackProjX;
    float mTrackProjY;
    float mTrackViewCos;

    // Scale invariance distances
    float mfMinDistance;
    float mfMaxDistance;

    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;

    map<int, size_t> mObservations;  //cameraId, index

    int PredictScale(float currentDist)
    {
        float ratio;

        ratio = mfMaxDistance/currentDist;

        int nScale = ceil(log(ratio)/mfLogScaleFactor);
        if(nScale<0)
            nScale = 0;
        else if(nScale>=mnScaleLevels)
            nScale = mnScaleLevels-1;

        return nScale;
    }

    void AddObservation(int cameraId, size_t idx)
    {
        if(mObservations.count(cameraId))
            return;
        // 记录下能观测到该MapPoint的KF和该MapPoint在KF中的索引
        mObservations[cameraId]=idx;
    }

    void ComputeDistinctiveDescriptors()
    {
        // Retrieve all observed descriptors
        vector<cv::Mat> vDescriptors;

        map<int, size_t> observations;


        observations=mObservations;

        if(observations.empty())
            return;

        vDescriptors.reserve(observations.size());

        for(map<int,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            int cameraId = mit->first;


            vDescriptors.push_back(mvpDescriptors[cameraId].row(mit->second));
        }

        if(vDescriptors.empty())
            return;

        // Compute distances between them
        const size_t N = vDescriptors.size();

        float Distances[N][N];
        for(size_t i=0;i<N;i++)
        {
            Distances[i][i]=0;
            for(size_t j=i+1;j<N;j++)
            {
                int distij = DescriptorDistance(vDescriptors[i],vDescriptors[j]);
                Distances[i][j]=distij;
                Distances[j][i]=distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for(size_t i=0;i<N;i++)
        {
            vector<int> vDists(Distances[i],Distances[i]+N);
            sort(vDists.begin(),vDists.end());
            int median = vDists[0.5*(N-1)];

            if(median<BestMedian)
            {
                BestMedian = median;
                BestIdx = i;
            }
        }

        mDescriptor = vDescriptors[BestIdx].clone();
    }


    int DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist=0;

        for(int i=0; i<8; i++, pa++, pb++)
        {
            unsigned  int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }
};




//forward decs
class MapPoint;
class KeyFrame;
class BundledKeyFrames;
//--------------

class Frame : public boost::enable_shared_from_this<Frame>
{
public:
    typedef boost::shared_ptr<Frame> frameptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<BundledKeyFrames> bkfptr;
public:

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for Monocular cameras,加入了基线和远近点门限
    // Frame(const cv::Mat &imGray, const double &timeStamp, extractorptr pExtractor, vocptr pVoc, cv::Mat &K, cv::Mat &distCoef, size_t ClientId, const float &bf, const float &thDepth);

    //加入双目帧的构造函数
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, extractorptr extractorLeft, extractorptr extractorRight, vocptr pVoc, cv::Mat &K, cv::Mat &distCoef, size_t ClientId, const float &bf, const float &thDepth);

    //Constructor for Multi-cameras
    Frame(const vector<cv::Mat> &vImage, const double &timeStamp, vector<extractorptr> vExtractor, vocptr pVoc, vector<cv::Mat> &vK, vector<cv::Mat> &vDistCoef, size_t ClientId, const float &thDepth);
    // Extract ORB on the image.,加入判断符号，确定左右目的提取
    void ExtractORB(int flag, const cv::Mat &im);
    void ExtractORBForMulti(int flag, const cv::Mat &im);
    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation

    inline cv::Mat GetCameraCenter(const int &cameraId);
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(mpptr pMP, float viewingCosLimit);
    //for multi camera
    bool isInFrustum(const int &cameraId, mpptr pMP, float viewingCosLimit);

    //for multi camera in frame initialition
    bool isInFrustum(const int &cameraId, X3D &x3D);
    //for multi camera
    bool isInFrustum(const int &cameraId, MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY, const int &cameraId);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    //add for multi camera
    vector<size_t> GetFeaturesInArea(const int &cameraId, const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    //加入两个用于计算双目信息特征点匹配和深度信息的函数
    void ComputeStereoMatches();

    //for multi
    vector<X3D> ComputeMultiMatches();

    vector<X3D> CreateMapPoints(const int &cameraIdi, const int &cameraIdj, vector<set<int>> &vUpdatedKPIndex, vector<vector<int>> &mMapCamera, vector<cv::Mat> &mvJointDescriptors);

    void CreateJointDescriptors(const vector<X3D> &vmp, const int &nonMonoNum);  //for multi camera

    void ComputeBundledDepth(const vector<X3D> &vmp, const int &nonMonoNum);

    int BilinearInterpolation(const float &x, const float &y, const cv::Mat &image);

    cv::Mat RemapSlidingWindow(const float &x, const float &y, const cv::Mat &image, const float &thita);

    int SearchByProjection(const int &cameraIdj, vector<X3D> &vx3Ds, const float th, vector<set<int>> &vUpdatedKPIndex,  vector<vector<int>> &vKeyPoinsIndexMap, const float &mfNNratio);
    
    cv::Mat ComputeDepth(const int &cameraIdi, const int &cameraIdj, const float &u1, const float &v1, const float &u2, const float &v2);

    cv::Mat UnprojectStereo(const int &i);
    
    cv::Mat ComputeMapPointInWorldFrame(const cv::Mat &x3D);

public:
    //add  new parameter for multi
    int cameraNum;

    vector<X3D> mvx3Ds;

    int nonMonoNum;

    // Vocabulary used for relocalization.
    vocptr mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    extractorptr mpORBextractor, mpORBextractorRight ;

    // Feature extractor for muli-camera case
    vector<extractorptr> mvpORBextractor;
    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    vector<cv::Mat> mvpK;

    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;

    //for multi camera;
    static vector<float> vfx;
    static vector<float> vfy;
    static vector<float> vcx;
    static vector<float> vcy;
    static vector<float> vinvfx;
    static vector<float> vinvfy;

    cv::Mat mDistCoef;
    vector<cv::Mat> mvpDistCoef;


    cv::Mat mTrl;
    std::vector<cv::Mat> mvTcamji;
    // Number of KeyPoints.
    int N;

    int N_L_R;

    vector<int> mvpkeyPointsNum;

    //加入fx乘以基线的长度
    float mbf;

    //加入远近点门限和基线长度
    float mb;
    float mThDepth;
    std::vector<vector<int>> vKeyPointsIndexMapPlus; //  if 400 ^ 500 = 300 vKeyPoinsIndexMap size is 600 for 4 camera

    std::vector<float> mvBDepth;   // size 600;
    cv::Mat mJointDescriptors;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys , mvKeysRight;
    //multi
    std::vector<vector<cv::KeyPoint>> mvKeysMultiple;
    std::vector<cv::KeyPoint> mvKeysUn;
    std::vector<vector<cv::KeyPoint>> mvKeysMultipleUn; //multi
    // 对于双目，mvuRight存储了左目像素点在右目中的对应点的横坐标
    // mvDepth对应的深度
    // 单目摄像头，这两个容器中存的都是-1
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    //Add for multi-camera
    // vector<DBoW2::BowVector> mvpBowVec;
    // vector<DBoW2::FeatureVector> mvpFeatVec;

    DBoW2::BowVector mBowVecBundled;
    DBoW2::FeatureVector mFeatVecBundled;

    // ORB descriptor, each row associated to a keypoint.,加入右目的描述子
    cv::Mat mDescriptors, mDescriptorsRight;
    //for multi-camera descriptors
    vector<cv::Mat> mvpDescriptors;
    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<mpptr> mvpMapPoints;
    std::vector<mpptr> mvpMapPointsBKFs;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;
    std::vector<bool> mvbOutlierBKFs;
    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    //for multi camera
    static vector<float> mvGridElementWidthInv;
    static vector<float> mvGridElementHeightInv;
    std::vector<std::size_t> mGrid0[FRAME_GRID_COLS][FRAME_GRID_ROWS];
    std::vector<std::size_t> mGrid1[FRAME_GRID_COLS][FRAME_GRID_ROWS];
    std::vector<std::size_t> mGrid2[FRAME_GRID_COLS][FRAME_GRID_ROWS];
    std::vector<std::size_t> mGrid3[FRAME_GRID_COLS][FRAME_GRID_ROWS];
    //end
    
    // Camera pose.
    cv::Mat mTcw;


    // Current and Next Frame id.
    static long unsigned int nNextId;
    idpair mId;

    // Reference Keyframe.
    kfptr mpReferenceKF;
    bkfptr mpReferenceBKFs;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    //add  for multi
    static vector<float> mvMinX;
    static vector<float> mvMaxX;
    static vector<float> mvMinY;
    static vector<float> mvMaxY;

    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();
    void UndistortKeyPointsForMultiCamera();
    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    //for multi
    void ComputeImageBounds(const int &cameraId, const cv::Mat &im);
    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();
    //add for multi camera
    void AssignFeaturesToGrid(const int &cameraId);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    pair<bool, vector<float>> ComputeEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12);

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

} //end namespace


#endif
