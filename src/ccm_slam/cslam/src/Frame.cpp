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

//Frame.cpp文件中来看，与ORB-SLAM2不一样的地方只有在构造函数，特征提取，帧号保存，以及只保存了和单目相关的成员变量和成员函数
#include <cslam/Frame.h>

namespace cslam {

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;
//for multi camera
vector<float> Frame::vcx, Frame::vcy, Frame::vfx, Frame::vfy, Frame::vinvfx, Frame::vinvfy;
vector<float> Frame::mvMinX, Frame::mvMaxX, Frame::mvMinY,  Frame::mvMaxY;
vector<float> Frame::mvGridElementWidthInv, Frame::mvGridElementHeightInv;

//Copy Constructor
//缺少构造双目的一些变量
// Frame::Frame(const Frame &frame)
//     :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractor(frame.mpORBextractor), mpORBextractorRight(frame.mpORBextractorRight),
//      mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
//      mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys), 
//      mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn), mvuRight(frame.mvuRight),
//      mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec), 
//      mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()), 
//      mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mId(frame.mId),
//      mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
//      mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
//      mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
//      mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
// {
//     for(int i=0;i<FRAME_GRID_COLS;i++)
//         for(int j=0; j<FRAME_GRID_ROWS; j++)
//             mGrid[i][j]=frame.mGrid[i][j];

//     if(!frame.mTcw.empty())
//         SetPose(frame.mTcw);
// }


Frame::Frame(const Frame &frame):
    cameraNum(frame.cameraNum),mvx3Ds(frame.mvx3Ds), mpORBvocabulary(frame.mpORBvocabulary), mvpORBextractor(frame.mvpORBextractor),
    mTimeStamp(frame.mTimeStamp), mvpK(frame.mvpK), mvpDistCoef(frame.mvpDistCoef),
    mThDepth(frame.mThDepth), mvTcamji(frame.mvTcamji), N_L_R(frame.N_L_R), vKeyPointsIndexMapPlus(frame.vKeyPointsIndexMapPlus),
    mvBDepth(frame.mvBDepth), mJointDescriptors(frame.mJointDescriptors.clone()),mvKeysMultiple(frame.mvKeysMultiple),
    mvKeysMultipleUn(frame.mvKeysMultipleUn),mvuRight(frame.mvuRight),
    mvDepth(frame.mvDepth),
    mBowVecBundled(frame.mBowVecBundled), mFeatVecBundled(frame.mFeatVecBundled),
    mvpDescriptors(frame.mvpDescriptors),
    mvpMapPointsBKFs(frame.mvpMapPointsBKFs),mvbOutlierBKFs(frame.mvbOutlierBKFs), mId(frame.mId),
    mpReferenceBKFs(frame.mpReferenceBKFs),mnScaleLevels(frame.mnScaleLevels),
    mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
    mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
    mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{

    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid0[i][j]=frame.mGrid0[i][j];

    //Add
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid1[i][j]=frame.mGrid1[i][j];

//    for(int i=0;i<FRAME_GRID_COLS;i++)
//        for(int j=0; j<FRAME_GRID_ROWS; j++)
//            mGrid2[i][j]=frame.mGrid2[i][j];
//
//    for(int i=0;i<FRAME_GRID_COLS;i++)
//        for(int j=0; j<FRAME_GRID_ROWS; j++)
//            mGrid3[i][j]=frame.mGrid3[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);

}   
    

//双目构造函数
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, extractorptr extractorLeft, extractorptr extractorRight, vocptr pVoc, cv::Mat &K, cv::Mat &distCoef, size_t ClientId, const float &bf, const float &thDepth)
    :mpORBvocabulary(pVoc), mpORBextractor(extractorLeft), mpORBextractorRight(extractorRight),mTimeStamp(timeStamp), 
    mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),  mpReferenceKF(nullptr)
{

    mId = make_pair(nNextId++,ClientId);

    // Scale Level Info
    mnScaleLevels = mpORBextractor->GetLevels();
    mfScaleFactor = mpORBextractor->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractor->GetScaleFactors();
    mvInvScaleFactors = mpORBextractor->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractor->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractor->GetInverseScaleSigmaSquares();

    // 同时对左右目提特征
    // ExtractORB(0, imLeft);
    // ExtractORB(1, imRight);

    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    
    threadLeft.join();
    threadRight.join();    

    N = mvKeys.size();

    if(mvKeys.empty())
        return;
    // Undistort特征点，这里没有对双目进行校正，因为要求输入的图像已经进行极线校正
    UndistortKeyPoints();

    // 计算双目间的匹配, 匹配成功的特征点会计算其深度
    // 深度存放在mvuRight 和 mvDepth 中
    ComputeStereoMatches();

    mvpMapPoints = vector<mpptr>(N,nullptr);
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }
    for(int id = 0 ; id < cameraNum; id++)
        AssignFeaturesToGrid(id);

   
}

//for multi-camera
Frame:: Frame(const vector<cv::Mat> &vImage, const double &timeStamp, vector<extractorptr> vExtractor, vocptr pVoc, vector<cv::Mat> &vK, vector<cv::Mat> &vDistCoef, size_t ClientId, const float &thDepth)
    :mpORBvocabulary(pVoc), mvpORBextractor(vExtractor), mTimeStamp(timeStamp), mvpK(vK), mvpDistCoef(vDistCoef), mThDepth(thDepth), mpReferenceBKFs(nullptr)
{
    
    mId = make_pair(nNextId++,ClientId);
    long temp_time = long(mTimeStamp*100 - 140363650000);
    cout<<"第"<<mId.first<<"帧时间戳 :"<<temp_time<<endl;
    //debug
    // long temp_time = long(mTimeStamp*100 - 140363650000);
    // cout<<"第"<<mId.first<<"帧时间戳 :"<<temp_time<<endl;
    
    cameraNum = vImage.size();
    cout<<"Frame camera num: "<<cameraNum<<endl;
    mvKeysMultiple.resize(cameraNum);
    mvKeysMultipleUn.resize(cameraNum);
    mvpDescriptors.resize(cameraNum);

    // Scale Level Info
    mnScaleLevels = mvpORBextractor[0]->GetLevels();
    mfScaleFactor = mvpORBextractor[0]->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mvpORBextractor[0]->GetScaleFactors();
    mvInvScaleFactors = mvpORBextractor[0]->GetInverseScaleFactors();
    mvLevelSigma2 = mvpORBextractor[0]->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mvpORBextractor[0]->GetInverseScaleSigmaSquares();
   
   //extrinisic
    mb = 47.90639384423901/435.2046959714599;
    mTrl = cv::Mat::eye(4,4,CV_32F);
    mTrl.at<float>(0,3)=-mb;
    mvTcamji.push_back(mTrl);

    // ORB extraction
    thread thread_1(&Frame::ExtractORBForMulti,this,0, vImage[0]);
    thread thread_2(&Frame::ExtractORBForMulti,this,1, vImage[1]);
    //thread thread_3(&Frame::ExtractORBForMulti,this,2, vImage[2]);
    //thread thread_4(&Frame::ExtractORBForMulti,this,3, vImage[3]);
   
    
    thread_1.join();
    thread_2.join();   

    mvpkeyPointsNum.resize(cameraNum);

    vfx.resize(cameraNum);
    vfy.resize(cameraNum);
    vcx.resize(cameraNum);
    vcy.resize(cameraNum);
    vinvfx.resize(cameraNum);
    vinvfy.resize(cameraNum);

    mvMinX.resize(cameraNum);
    mvMaxX.resize(cameraNum);
    mvMinY.resize(cameraNum);
    mvMaxY.resize(cameraNum);

    mvGridElementWidthInv.resize(cameraNum);
    mvGridElementHeightInv.resize(cameraNum);

    for(int i = 0; i < cameraNum; i++)
    {
        mvpkeyPointsNum[i]=mvKeysMultiple[i].size();
    }

    if(mvKeysMultiple[0].empty() || mvKeysMultiple[1].empty()) //||mvKeysMultiple[2].empty()||mvKeysMultiple[3].empty()
        return;

    UndistortKeyPointsForMultiCamera();
   
    mvx3Ds = ComputeMultiMatches();
 
    N_L_R = vKeyPointsIndexMapPlus.size();
    
    ComputeBundledDepth(mvx3Ds, nonMonoNum);

    mvpMapPointsBKFs = vector<mpptr>(N_L_R,nullptr);
    mvbOutlierBKFs = vector<bool>(N_L_R, false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        for (int num = 0; num < cameraNum; num++)
        {
            ComputeImageBounds(num, vImage[num]);

            mvGridElementWidthInv[num] = static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mvMaxX[num]-mvMinX[num]);
            mvGridElementHeightInv[num] = static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mvMaxY[num]-mvMinY[num]);

            vfx[num] = vK[num].at<float>(0,0);
            vfy[num] = vK[num].at<float>(1,1);
            vcx[num] = vK[num].at<float>(0,2);
            vcy[num] = vK[num].at<float>(1,2);
            vinvfx[num] = 1.0f/vfx[num];
            vinvfy[num] = 1.0f/vfy[num];
        }


        mbInitialComputations=false;
    }

    mb = mbf/fx;
    for(int id = 0 ; id < cameraNum; id++)
        AssignFeaturesToGrid(id);

    cout<<"Frame init sucess"<<endl;
}
/*
//单目相机，构造函数里面与ORB-SLAM中有些不一样，帧的ID使用的是pair来存储的，与ClientId进行对应
Frame::Frame(const cv::Mat &imGray, const double &timeStamp, extractorptr pExtractor, vocptr pVoc, cv::Mat &K, cv::Mat &distCoef, size_t ClientId, const float &bf, const float &thDepth)
// Frame::Frame(const cv::Mat &imGray, const double &timeStamp, extractorptr pExtractor, vocptr pVoc, cv::Mat &K, cv::Mat &distCoef, size_t ClientId)
    :mpORBvocabulary(pVoc),mpORBextractor(pExtractor), mpORBextractorRight(nullptr),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{   
    //构造函数里面与ORB-SLAM中有些不一样，帧的ID使用的是pair来存储的，与ClientId进行对应
    // Frame ID
    mId = make_pair(nNextId++,ClientId);
    // Scale Level Info
    mnScaleLevels = mpORBextractor->GetLevels();
    mfScaleFactor = mpORBextractor->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractor->GetScaleFactors();
    mvInvScaleFactors = mpORBextractor->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractor->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractor->GetInverseScaleSigmaSquares();

    // ORB extraction。只对单目，所以把ORB-SLAM2中的判断左右目图像进行了删除
    ExtractORB(0, imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    //不存储双目的信息
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<mpptr>(N,nullptr);
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx; 
    AssignFeaturesToGrid();
}
*/

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}


void Frame::AssignFeaturesToGrid(const int &cameraId)
{
    int nReserve = 0.5f*mvpkeyPointsNum[cameraId]/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    if(cameraId == 0)
    {
        for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
            for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
                mGrid0[i][j].reserve(nReserve);


        for(int i=0;i<mvpkeyPointsNum[cameraId];i++)
        {
            const cv::KeyPoint &kp = mvKeysMultipleUn[cameraId][i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY, cameraId))
                mGrid0[nGridPosX][nGridPosY].push_back(i);
        }
    }
    else if(cameraId == 1)
    {
        for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
            for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
                mGrid1[i][j].reserve(nReserve);

        for(int i=0;i<mvpkeyPointsNum[cameraId];i++)
        {
            const cv::KeyPoint &kp = mvKeysMultipleUn[cameraId][i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY, cameraId))
                mGrid1[nGridPosX][nGridPosY].push_back(i);
        }
    }
    else if(cameraId == 2)
    {
        for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
            for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
                mGrid2[i][j].reserve(nReserve);

        for(int i=0;i<mvpkeyPointsNum[cameraId];i++)
        {
            const cv::KeyPoint &kp = mvKeysMultipleUn[cameraId][i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY, cameraId))
                mGrid2[nGridPosX][nGridPosY].push_back(i);
        }
    }
    else
    {
        for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
            for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
                mGrid3[i][j].reserve(nReserve);

        for(int i=0;i<mvpkeyPointsNum[cameraId];i++)
        {
            const cv::KeyPoint &kp = mvKeysMultipleUn[cameraId][i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY, cameraId))
                mGrid3[nGridPosX][nGridPosY].push_back(i);
        }
    }
}


//对这一部分进行了修改，只有单目的特征提取,变成双目提取
void Frame::ExtractORB(int flag, const cv::Mat &im)
{   
    if(flag==0)
        (*mpORBextractor)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
    
}

void Frame::ExtractORBForMulti(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mvpORBextractor[0])(im,cv::Mat(),mvKeysMultiple[0],mvpDescriptors[0]);
    else if(flag==1)
        (*mvpORBextractor[1])(im,cv::Mat(),mvKeysMultiple[1],mvpDescriptors[1]);
    else if(flag==2)
        (*mvpORBextractor[2])(im,cv::Mat(),mvKeysMultiple[2],mvpDescriptors[2]);
    else
        (*mvpORBextractor[3])(im,cv::Mat(),mvKeysMultiple[3],mvpDescriptors[3]);
}


void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;
}

bool Frame::isInFrustum(mpptr pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos();

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
    {
        return false;
    }

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this->shared_from_this());

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz; //该3D点投影到双目右侧相机上的横坐标
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

bool Frame::isInFrustum(const int &cameraId, mpptr pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat Pw = pMP->GetWorldPos();

    cv::Mat tTcami0= cv::Mat::eye(4,4,CV_32F); //T10
    for(int x = 0; x < cameraId; x++)
    {
        tTcami0 = mvTcamji[x]* tTcami0;  //Ti0
    }

    cv::Mat tTiw;
    tTiw = tTcami0 * mTcw;  //Tiw

    cv::Mat Riw = tTiw.rowRange(0,3).colRange(0,3);
    cv::Mat tiw = tTiw.rowRange(0,3).col(3);


    // 3D in camera coordinates
    const cv::Mat Pc = Riw*Pw+tiw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=vfx[cameraId]*PcX*invz+vcx[cameraId];
    const float v=vfy[cameraId]*PcY*invz+vcy[cameraId];

    if(u<mvMinX[cameraId] || u>mvMaxX[cameraId])
        return false;
    if(v<mvMinY[cameraId] || v>mvMaxY[cameraId])
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();

    const cv::Mat twi = GetCameraCenter(cameraId);


    const cv::Mat PO = Pw-twi;  // 世界坐标系下，相机到3D点P的向量, 向量方向由相机(i)指向3D点P
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
    {
        return false;
    }

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this->shared_from_this());

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}



inline cv::Mat Frame::GetCameraCenter(const int & cameraId)
{

    cv::Mat tTcami0= cv::Mat::eye(4,4,CV_32F); //T10
    for(int x = 0; x < cameraId; x++)
    {
        tTcami0 = mvTcamji[x]* tTcami0;  //Ti0
    }

    cv::Mat tTiw;
    tTiw = tTcami0 * mTcw;

    cv::Mat tRiw = tTiw.rowRange(0,3).colRange(0,3);
    cv::Mat ttiw = tTiw.rowRange(0,3).col(3);
    cv::Mat ttwi = -tRiw.t()*ttiw;  //mtwi
    return ttwi.clone();
}


bool Frame::isInFrustum(const int &cameraId, X3D &x3D)
{
    
// 3D in absolute coordinates
    cv::Mat P0 = x3D.mPos;

    cv::Mat t0c = cv::Mat::zeros(3,1, CV_32F);

    cv::Mat Pc;
    if(cameraId == 0)
        Pc = P0;

    else if(cameraId == 1)
    {
        cv::Mat R10 = mvTcamji[0].rowRange(0,3).colRange(0,3);
        cv::Mat t10 = mvTcamji[0].rowRange(0,3).col(3);
        Pc = R10*P0+t10;
        t0c = -R10.t() * t10;
    }
    else if(cameraId == 2)
    {
        cv::Mat R10 = mvTcamji[0].rowRange(0,3).colRange(0,3);
        cv::Mat t10 = mvTcamji[0].rowRange(0,3).col(3);

        cv::Mat R21 = mvTcamji[1].rowRange(0,3).colRange(0,3);
        cv::Mat t21 = mvTcamji[1].rowRange(0,3).col(3);
        Pc = R10*P0+t10;
        Pc = R21 * Pc + t21;

        cv::Mat tTji = mvTcamji[1] *mvTcamji[0];
        cv::Mat R20 = tTji.rowRange(0,3).colRange(0,3);
        cv::Mat t20 = tTji.rowRange(0,3).col(3);
        t0c = -R20.t()*t20;
    }
    else
    {
        cv::Mat R10 = mvTcamji[0].rowRange(0,3).colRange(0,3);
        cv::Mat t10 = mvTcamji[0].rowRange(0,3).col(3);

        cv::Mat R21 = mvTcamji[1].rowRange(0,3).colRange(0,3);
        cv::Mat t21 = mvTcamji[1].rowRange(0,3).col(3);

        cv::Mat R32 = mvTcamji[2].rowRange(0,3).colRange(0,3);
        cv::Mat t32 = mvTcamji[2].rowRange(0,3).col(3);

        Pc = R10*P0+t10;
        Pc = R21 * Pc + t21;
        Pc = R32* Pc + t32;

        cv::Mat tTji = mvTcamji[2] * mvTcamji[1] *mvTcamji[0];
        cv::Mat R30 = tTji.rowRange(0,3).colRange(0,3);
        cv::Mat t30 = tTji.rowRange(0,3).col(3);
        t0c = -R30.t()*t30;

    }

    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float currentfx = mvpK[cameraId].at<float>(0,0);
    const float currentfy = mvpK[cameraId].at<float>(1,1);
    const float currentcx = mvpK[cameraId].at<float>(0,2);
    const float currentcy = mvpK[cameraId].at<float>(1,2);

    const float u=currentfx*PcX*invz+currentcx;
    const float v=currentfy*PcY*invz+currentcy;

    if(u<mvMinX[cameraId] || u>mvMaxX[cameraId])
        return false;
    if(v<mvMinY[cameraId] || v>mvMaxY[cameraId])
        return false;

    //看到改mappoint的子相机和第0号子相机的光心距离
    cv::Mat Oc;
    cv::Mat tTji = cv::Mat::eye(4,4,CV_32F);
    for(int i = x3D.cameraId-1; i>=0; i--)
    {
        tTji = mvTcamji[i] * tTji;
    }
    cv::Mat Rc0 = tTji.rowRange(0,3).colRange(0,3);
    cv::Mat tc0 = tTji.rowRange(0,3).col(3);
    Oc = -Rc0.t()*tc0;


    cv::Mat PC = x3D.mPos - Oc;
    const float dis = cv::norm(PC);
    const int level = mvKeysMultipleUn[x3D.cameraId][x3D.keypointId].octave;
    const float levelScaleFactor = mvScaleFactors[level];
    const int nLevels = mnScaleLevels;

    float tmfMaxDistance = dis*levelScaleFactor;
    float tmfMinDistance = tmfMaxDistance/mvScaleFactors[nLevels-1];


    const float maxDistance = 1.2f * tmfMaxDistance;
    const float minDistance = 0.8f * tmfMinDistance;

    const cv::Mat PO = P0-t0c;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

    // Check viewing angle
    x3D.mTrackProjX = u;
    x3D.mTrackProjY = v;

    return true;


}



vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

vector<size_t> Frame::GetFeaturesInArea(const int  &cameraId, const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(mvpkeyPointsNum[cameraId]);

    const int nMinCellX = max(0,(int)floor((x-mvMinX[cameraId]-r)*mvGridElementWidthInv[cameraId]));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mvMinX[cameraId]+r)*mvGridElementWidthInv[cameraId]));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mvMinY[cameraId]-r)*mvGridElementHeightInv[cameraId]));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mvMinY[cameraId]+r)*mvGridElementHeightInv[cameraId]));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);


    vector<std::size_t> tGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];


    if(cameraId == 0)
        std::copy(&mGrid0[0][0], &mGrid0[0][0]+FRAME_GRID_COLS*FRAME_GRID_ROWS,&tGrid[0][0]);
    else if(cameraId == 1)
        std::copy(&mGrid1[0][0], &mGrid1[0][0]+FRAME_GRID_COLS*FRAME_GRID_ROWS,&tGrid[0][0]);
    else if(cameraId == 2)
        std::copy(&mGrid2[0][0], &mGrid2[0][0]+FRAME_GRID_COLS*FRAME_GRID_ROWS,&tGrid[0][0]);
    else
        std::copy(&mGrid3[0][0], &mGrid3[0][0]+FRAME_GRID_COLS*FRAME_GRID_ROWS,&tGrid[0][0]);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = tGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysMultipleUn[cameraId][vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}




bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY, const int &cameraId)
{

    posX = round((kp.pt.x - mvMinX[cameraId])*mvGridElementWidthInv[cameraId]);
    posY = round((kp.pt.y - mvMinY[cameraId])*mvGridElementHeightInv[cameraId]);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;

}

void Frame::ComputeBoW()
{
    // if(mBowVec.empty())
    // {
    //     vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
    //     mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    // }
    
    //revise here for multi-camera
    if(mBowVecBundled.empty())
    {
        vector<cv::Mat> vCurrentDescBundled= Converter::toDescriptorVector(mJointDescriptors);
        mpORBvocabulary->transform(vCurrentDescBundled,mBowVecBundled,mFeatVecBundled,4);

    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        //cout<<"图像已经矫正，跳过后面的操作！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！"<<endl;
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::UndistortKeyPointsForMultiCamera()
{
    
    if(mvpDistCoef[0].at<float>(0)==0.0)
    {
        mvKeysMultipleUn[0] = mvKeysMultiple[0];
        mvKeysMultipleUn[1] = mvKeysMultiple[1];
        //do not forget to add 2,3 camera
        //todo:
        return;
    }


    for(int i = 0; i < cameraNum; i++)
    {
        // Fill matrix with points
        int N = mvKeysMultiple[i].size();
        cv::Mat mat(N,2,CV_32F);
        for(int j=0; j<N; j++)
        {
            mat.at<float>(i,0)=mvKeysMultiple[i][j].pt.x;
            mat.at<float>(i,1)=mvKeysMultiple[i][j].pt.y;
        }

        // Undistort points
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mvpK[i],mvpDistCoef[i],cv::Mat(),mvpK[i]);
        mat=mat.reshape(1);

        // Fill undistorted keypoint vector
        mvKeysMultipleUn[i].resize(N);
        for(int j=0; j<N; j++)
        {
            cv::KeyPoint kp = mvKeysMultiple[i][j];
            kp.pt.x=mat.at<float>(j,0);
            kp.pt.y=mat.at<float>(j,1);
            mvKeysMultipleUn[i][j]=kp;
        }
    }


}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; 
        mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols;
        mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; 
        mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; 
        mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeImageBounds(const int &cameraId, const cv::Mat &im)
{
    if(mvpDistCoef[cameraId].at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=im.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=im.rows;
        mat.at<float>(3,0)=im.cols; mat.at<float>(3,1)=im.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mvpK[cameraId],mvpDistCoef[cameraId],cv::Mat(),mvpK[cameraId]);
        mat=mat.reshape(1);

        mvMinX[cameraId] = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mvMaxX[cameraId] = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mvMinY[cameraId] = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mvMaxY[cameraId] = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mvMinX[cameraId] = 0.0f;
        mvMaxX[cameraId] = im.cols;
        mvMinY[cameraId] = 0.0f;
        mvMaxY[cameraId] = im.rows;
    }
}

/* @brief 双目匹配
 *
 * 为左图的每一个特征点在右图中找到匹配点 \n
 * 根据基线(有冗余范围)上描述子距离找到匹配, 再进行SAD精确定位 \n
 * 最后对所有SAD的值进行排序, 剔除SAD值较大的匹配对，然后利用抛物线拟合得到亚像素精度的匹配 \n
 * 匹配成功后会更新 mvuRight(ur) 和 mvDepth(Z)
 */
void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractor->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    // 步骤1：建立特征点搜索范围对应表，一个特征点在一个带状区域内搜索匹配特征点
    // 匹配搜索的时候，不仅仅是在一条横线上搜索，而是在一条横向搜索带上搜索,简而言之，原本每个特征点的纵坐标为1，这里把特征点体积放大，纵坐标占好几行
    // 例如左目图像某个特征点的纵坐标为20，那么在右侧图像上搜索时是在纵坐标为18到22这条带上搜索，搜索带宽度为正负2，搜索带的宽度和特征点所在金字塔层数有关
    // 简单来说，如果纵坐标是20，特征点在图像第20行，那么认为18 19 20 21 22行都有这个特征点
    // vRowIndices[18]、vRowIndices[19]、vRowIndices[20]、vRowIndices[21]、vRowIndices[22]都有这个特征点编号
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        // !!在这个函数中没有对双目进行校正，双目校正是在外层程序中实现的
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        // 计算匹配搜索的纵向宽度，尺度越大（层数越高，距离越近），搜索范围越大
        // 如果特征点在金字塔第一层，则搜索范围为:正负2
        // 尺度越大其位置不确定性越高，所以其搜索半径越大
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;        // NOTE bug mb没有初始化，mb的赋值在构造函数中放在ComputeStereoMatches函数的后面
    const float minD = 0;        // 最小视差, 设置为0即可
    const float maxD = mbf/minZ;  // 最大视差, 对应最小深度 mbf/minZ = mbf/mb = mbf/(mbf/fx) = fx (wubo???)

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    // 步骤2：对左目相机每个特征点，通过描述子在右目带状搜索区域找到匹配点, 再通过SAD做亚像素匹配
    // 注意：这里是校正前的mvKeys，而不是校正后的mvKeysUn
    // KeyFrame::UnprojectStereo和Frame::UnprojectStereo函数中不一致
    // 这里是不是应该对校正后特征点求深度呢？(wubo???)
    //int ncount = 0;
    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        // 可能的匹配点
        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD; // 最小匹配范围
        const float maxU = uL-minD; // 最大匹配范围

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        // 每个特征点描述子占一行，建立一个指针指向iL特征点对应的描述子
        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        // 步骤2.1：遍历右目所有可能的匹配点，找出最佳匹配点（描述子距离最小）
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            // 仅对近邻尺度的特征点进行匹配
            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }
        // 最好的匹配的匹配误差存在bestDist，匹配点位置存在bestIdxR中

        // Subpixel match by correlation
        // 步骤2.2：通过SAD匹配提高像素匹配修正量bestincR
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            // kpL.pt.x对应金字塔最底层坐标，将最佳匹配的特征点对尺度变换到尺度对应层 (scaleduL, scaledvL) (scaleduR0, )
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5; // 滑动窗口的大小11*11 注意该窗口取自resize后的图像
            cv::Mat IL = mpORBextractor->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);//窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1); // 11

            // 滑动窗口的滑动范围为（-L, L）,提前判断滑动窗口滑动过程中是否会越界
            const float iniu = scaleduR0+L-w; //这个地方是否应该是scaleduR0-L-w (wubo???)
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                // 横向滑动窗口
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) * cv::Mat::ones(IR.rows,IR.cols,CV_32F);//窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响

                float dist = cv::norm(IL,IR,cv::NORM_L1); // 一范数，计算差的绝对值
                if(dist<bestDist)
                {
                    bestDist =  dist;// SAD匹配目前最小匹配偏差
                    bestincR = incR; // SAD匹配目前最佳的修正量
                }

                vDists[L+incR] = dist; // 正常情况下，这里面的数据应该以抛物线形式变化
            }

            if(bestincR==-L || bestincR==L) // 整个滑动窗口过程中，SAD最小值不是以抛物线形式出现，SAD匹配失败，同时放弃求该特征点的深度
                continue;

            // Sub-pixel match (Parabola fitting)
            // 步骤2.3：做抛物线拟合找谷底得到亚像素匹配deltaR
            // (bestincR,dist) (bestincR-1,dist) (bestincR+1,dist)三个点拟合出抛物线
            // bestincR+deltaR就是抛物线谷底的位置，相对SAD匹配出的最小值bestincR的修正量为deltaR
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            // 抛物线拟合得到的修正量不能超过一个像素，否则放弃求该特征点的深度
            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            // 通过描述子匹配得到匹配点位置为scaleduR0
            // 通过SAD匹配找到修正量bestincR
            // 通过抛物线拟合找到亚像素修正量deltaR
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            // 这里是disparity，根据它算出depth
            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD) // 最后判断视差是否在范围内
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                // if(mbf/disparity > 40*mb)
                // {
                //     ++ncount;
                //     continue;
                // }
                // depth 是在这里计算的
                // depth=baseline*fx/disparity
                mvDepth[iL]=mbf/disparity;   // 深度
                mvuRight[iL] = bestuR;       // 匹配对在右图的横坐标
                vDistIdx.push_back(pair<int,int>(bestDist,iL)); // 该特征点SAD匹配最小匹配偏差
            }
        }
    }
    //cout<<"+++++++++ ncount(大于基线40倍)："<<ncount<<endl;
    // 步骤3：剔除SAD匹配偏差较大的匹配特征点
    // 前面SAD匹配只判断滑动窗口中是否有局部最小值，这里通过对比剔除SAD匹配偏差比较大的特征点的深度
    sort(vDistIdx.begin(),vDistIdx.end()); // 根据所有匹配对的SAD偏差进行排序, 距离由小到大
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median; // 计算自适应距离, 大于此距离的匹配对将剔除

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}


vector<X3D> Frame::ComputeMultiMatches()
{
    //cout<<"Enter ComputeMultiMatches"<<endl;
    vector<set<int>> vUpdatedKPIndex(cameraNum, set<int>());
  
    for(int i  = 0; i < cameraNum; i++)
    {
        set<int> st;
        for(int j = 0; j < mvpkeyPointsNum[i]; j++)
            st.insert(j);
        vUpdatedKPIndex[i]= st;
    }

    vector<X3D> vmp; //in no.0 camera frame

    vector<vector<int>> vKeyPoinsIndexMap;

    vector<cv::Mat> mvJointDescriptors;

    for(int i = 0; i < cameraNum-1; i++)
    {
        for(int j = i+1; j < cameraNum; j++)
        {
            //step 1
            // if(vmp.empty())
            //     cout<<"-----Frame Empty -------"<<endl;
            if(!vmp.empty())
            {
                SearchByProjection(j, vmp, 1, vUpdatedKPIndex, vKeyPoinsIndexMap, 0.8);  //0.8 may change, we need to adjust this parameter
                cout<<"finish serachby projection"<<endl;
            }
            //step 2
            //cout<<"ComputeMultiMatches 2"<<endl;
            vector<X3D> x3Ds = CreateMapPoints(i, j, vUpdatedKPIndex, vKeyPoinsIndexMap, mvJointDescriptors);
    
            for(X3D &x3D : x3Ds)
            {
                // if(mId.first == 2)
                //     cout<<"----pos: "<<x3D.mPos<<endl;
                //cout<<"Frame init mp pos: "<<x3D.mPos<<endl;
                x3D.ComputeDistinctiveDescriptors();
                vmp.push_back(x3D);
            }

    
        }
    }

    nonMonoNum = vKeyPoinsIndexMap.size();

    //set mono point
    for(size_t i = 0; i < cameraNum; i++)
    {
        for(set<int>::iterator it = vUpdatedKPIndex[i].begin(); it != vUpdatedKPIndex[i].end(); it++)
        {
            vector<int> tmp(cameraNum, -1);
            tmp[i] = *it;
            vKeyPoinsIndexMap.push_back(tmp);
        }
    }

    vKeyPointsIndexMapPlus = vKeyPoinsIndexMap;

    CreateJointDescriptors(vmp, nonMonoNum);

    return vmp;
    
}


int Frame::SearchByProjection(const int &cameraIdj, vector<X3D> &vx3Ds, const float th, vector<set<int>> &vUpdatedKPIndex, vector<vector<int>> &vKeyPoinsIndexMap, const float &mfNNratio)
{
    int nmatches = 0;
    const int TH_HIGH = 100;

    for(size_t iMP = 0; iMP < vx3Ds.size(); iMP++)
    {
        // 3D in absolute coordinates
        X3D &pMP =  vx3Ds[iMP];

        cv::Mat P = pMP.mPos;
        //check whether this mappoint is in the current FOV
        if(!isInFrustum(cameraIdj, pMP))
            continue;

        cv::Mat t0c = cv::Mat::zeros(3,1, CV_32F);
        //predit the level
        if(cameraIdj == 1)
        {
            cv::Mat R10 = mvTcamji[0].rowRange(0,3).colRange(0,3);
            cv::Mat t10 = mvTcamji[0].rowRange(0,3).col(3);
            t0c = -R10.t() * t10;
        }
        else if(cameraIdj == 2)
        {
            cv::Mat tTji = mvTcamji[1] *mvTcamji[0];
            cv::Mat R20 = tTji.rowRange(0,3).colRange(0,3);
            cv::Mat t20 = tTji.rowRange(0,3).col(3);
            t0c = -R20.t()*t20;
        }
        else
        {
            cv::Mat tTji = mvTcamji[2] * mvTcamji[1] *mvTcamji[0];
            cv::Mat R30 = tTji.rowRange(0,3).colRange(0,3);
            cv::Mat t30 = tTji.rowRange(0,3).col(3);
            t0c = -R30.t()*t30;
        }



        cv::Mat PO = P - t0c;
        float dist3D = cv::norm(PO);

        int nPredictedLevel = pMP.PredictScale(dist3D);
        // The size of the window will depend on the viewing direction

        float r =  4.0;

        const vector<size_t> vIndices = GetFeaturesInArea(cameraIdj, pMP.mTrackProjX, pMP.mTrackProjY, r*mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel);

        if(vIndices.empty())
            continue;

        const cv::Mat MPdescriptor = pMP.mDescriptor;

        int bestDist=256;
        int bestLevel= -1;
        int bestDist2=256;
        int bestLevel2 = -1;
        int bestIdx =-1 ;

        // Get best and second matches with near keypoints
        for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        {
            const size_t idx = *vit;

            if(!vUpdatedKPIndex[cameraIdj].count(idx))
                continue;

            const cv::Mat &d = mvpDescriptors[cameraIdj].row(idx);

            const int dist = ORBmatcher::DescriptorDistance(MPdescriptor,d);

            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestLevel2 = bestLevel;
                bestLevel = mvKeysMultipleUn[cameraIdj][idx].octave;
                bestIdx=idx;
            }
            else if(dist<bestDist2)
            {
                bestLevel2 = mvKeysMultipleUn[cameraIdj][idx].octave;
                bestDist2=dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if(bestDist<=TH_HIGH)
        {
            if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
                continue;

            vUpdatedKPIndex[cameraIdj].insert(bestIdx);  //update keypoint

            if(vKeyPoinsIndexMap[iMP][cameraIdj] == -1)
                vKeyPoinsIndexMap[iMP][cameraIdj] = bestIdx;   //update mappoint

            pMP.AddObservation(cameraIdj, bestIdx);       //update this mappoint observation

            pMP.ComputeDistinctiveDescriptors();

            nmatches++;
        }

    }
    return nmatches;

}


vector<X3D> Frame::CreateMapPoints(const int &cameraIdi, const int &cameraIdj, vector<set<int>> &vUpdatedKPIndex, vector<vector<int>> &vKeyPoinsIndexMap, vector<cv::Mat> &mvJointDescriptors)
{
 
    vector<X3D> res;

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mvpORBextractor[cameraIdi]->mvImagePyramid[0].rows;

    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);


    for(int x = 0; x < mvpkeyPointsNum[cameraIdi]; x++)
    {

        if(!vUpdatedKPIndex[cameraIdi].count(x))
            continue;

        const cv::KeyPoint &kp1 = mvKeysMultipleUn[cameraIdi][x];
        const float &vL = kp1.pt.y;
        cv::Mat tTcamji= cv::Mat::eye(4,4,CV_32F);

        for(int index = cameraIdi; index < cameraIdj; index++)
        {
            tTcamji = mvTcamji[index]* tTcamji;  //R trangCamId2,trangCamId1
        }

        cv::Mat R21 = tTcamji.rowRange(0,3).colRange(0,3);  //R trangCamId2,trangCamId1
        cv::Mat R12 = R21.t();
        cv::Mat t21 =tTcamji.rowRange(0,3).col(3);
        cv::Mat t12 = -R12*t21;

        cv::Mat t12x = SkewSymmetricMatrix(t12);
        cv::Mat F12 = mvpK[cameraIdi].t().inv()*t12x*R12*mvpK[cameraIdj].inv();
        for(int y = 0; y <mvpkeyPointsNum[cameraIdj]; y++)
        {
            if(!vUpdatedKPIndex[cameraIdj].count(y))
                continue;

            const cv::KeyPoint &kp2 = mvKeysMultipleUn[cameraIdj][y];

            if(ComputeEpipolarLine(kp1, kp2, F12).first)
            {
                vRowIndices[vL].push_back(y);
            }
        }
    }

    //find matched keypoint
    for(int iL=0; iL<mvpkeyPointsNum[cameraIdi]; iL++)
    {

        if(!vUpdatedKPIndex[cameraIdi].count(iL))
            continue;
    
        const cv::KeyPoint &kpL = mvKeysMultipleUn[cameraIdi][iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if (vCandidates.empty())
            continue;
        //cout<<"FOUND ONE IN CANDADIATE"<<endl;
        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mvpDescriptors[cameraIdi].row(iL);

        // Compare descriptor to right keypoints
        for (size_t iC = 0; iC < vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysMultipleUn[cameraIdj][iR];

            if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
                continue;

            const float &uR = kpR.pt.x;

            const cv::Mat &dR = mvpDescriptors[cameraIdj].row(iR);
            const int dist = ORBmatcher::DescriptorDistance(dL, dR);

            if (dist < bestDist) {
                bestDist = dist;
                bestIdxR = iR;
            }

        }

        // Subpixel match by correlation
        if (bestDist < thOrbDist)
        {
            //cout<<"Enter CreateMapPoints "<<endl;
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysMultiple[cameraIdj][bestIdxR].pt.x;
            const float vR0 = mvKeysMultiple[cameraIdj][bestIdxR].pt.y;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x * scaleFactor);
            const float scaledvL = round(kpL.pt.y * scaleFactor);
            const float scaleduR0 = round(uR0 * scaleFactor);
            const float scaledvR0 = round(vR0 * scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mvpORBextractor[cameraIdi]->mvImagePyramid[kpL.octave].rowRange(scaledvL - w,scaledvL + w +1).colRange(scaleduL - w, scaleduL + w + 1);
            IL.convertTo(IL, CV_32F);
            IL = IL - IL.at<float>(w, w) * cv::Mat::ones(IL.rows, IL.cols, CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;

            vector<float> vDists;
            vDists.resize(2 * L + 1);

            const float iniu = scaleduR0 - L - w;
            const float endu = scaleduR0 + L + w + 1;
            if (iniu < 0 || endu >= mvpORBextractor[cameraIdj]->mvImagePyramid[kpL.octave].cols)
                continue;

            cv::Mat tTcamji = cv::Mat::eye(4, 4, CV_32F);

            for (int index = cameraIdi; index < cameraIdj; index++) {
                tTcamji = mvTcamji[index] * tTcamji;  //R trangCamId2,trangCamId1
            }

            cv::Mat R21 = tTcamji.rowRange(0, 3).colRange(0, 3);  //R trangCamId2,trangCamId1
            cv::Mat R12 = R21.t();
            cv::Mat t21 = tTcamji.rowRange(0, 3).col(3);
            cv::Mat t12 = -R12 * t21;

            cv::Mat t12x = SkewSymmetricMatrix(t12);
            cv::Mat F12 = mvpK[cameraIdi].t().inv() * t12x * R12 * mvpK[cameraIdj].inv();

            vector<float> vabc = ComputeEpipolarLine(kpL, mvKeysMultiple[1][bestIdxR], F12).second;

            float k = -1 * vabc[0] / vabc[1];
            float a = vabc[0], b = vabc[1], c = vabc[2];
            float deltax = sqrt(1.0 / (k * k + 1));
            float deltay = k * deltax;
            float thita = atan(k);
            for (int incR = -L; incR <= +L; incR++) {
                cv::Mat IR;
                if (a == 0.0)  //corner case
                {
                    IR = RemapSlidingWindow(scaleduR0 + incR * deltax, scaledvL + incR * deltay, mvpORBextractor[cameraIdj]->mvImagePyramid[kpL.octave],thita);
                }
                else if (b == 0.0) //corner case
                {
                    IR = mvpORBextractor[1]->mvImagePyramid[kpL.octave].rowRange(scaledvR0 - w,scaledvR0 + w + 1).colRange(scaleduL + incR - w, scaleduR0 + incR * deltax + w + 1);
                }
                else //general case
                {
                    float denominator = a * a + b * b;
                    float numerator = a * a * scaledvL - b * c - a * b * scaleduR0;
                    float vLProjectOnEpipolarLine = numerator / denominator;
                    IR = RemapSlidingWindow(scaleduR0 + incR * deltax, vLProjectOnEpipolarLine + incR * deltay, mvpORBextractor[cameraIdj]->mvImagePyramid[kpL.octave],thita);
                }

                IR.convertTo(IR, CV_32F);
                IR = IR - IR.at<float>(w, w) * cv::Mat::ones(IR.rows, IR.cols, CV_32F);
                float dist = cv::norm(IL, IR, cv::NORM_L1);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestincR = incR;
                }

                vDists[L + incR] = dist;
            }

            if (bestincR == -L || bestincR == L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L + bestincR - 1];
            const float dist2 = vDists[L + bestincR];
            const float dist3 = vDists[L + bestincR + 1];

            const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));
            
                //cout<<"dist: "<<dist1<<", "<<dist2<<", "<<dist3<<", "<<deltaR<<endl;
            if (deltaR < -1 || deltaR > 1)
                continue;
            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave] * ((float) scaleduR0 + (float) (bestincR + deltaR) * deltax);
            float bestvR = mvScaleFactors[kpL.octave] * ((float) scaledvR0 + (float) (bestincR + deltaR) * deltay);
            // if(mId.first ==2)
            //     cout << "uL: " << uL << ", " << bestuR;

            //update
            vUpdatedKPIndex[cameraIdi].erase(iL);
            vUpdatedKPIndex[cameraIdj].erase(bestIdxR);

            vector<int> tmp(cameraNum, -1);
            tmp[cameraIdi] = iL;
            tmp[cameraIdj] = bestIdxR;
            vKeyPoinsIndexMap.push_back(tmp);
            cv::Mat pos = ComputeDepth(cameraIdi, cameraIdj, uL, vL, bestuR, bestvR);
            // if(mId.first ==2)
            //     cout<<" depth: "<< pos.at<float>(2)<<endl;
            res.push_back({pos, cameraIdi, iL,mnScaleLevels, mfScaleFactor, mfLogScaleFactor, mvpDescriptors});
            res.rbegin()->AddObservation(cameraIdi, iL);
            res.rbegin()->AddObservation(cameraIdj, bestIdxR);
            //cout<<res.rbegin()->mPos<<endl;
        }
    }
    //cout<<"vKeyPoinsIndexMap: "<<vKeyPoinsIndexMap.size()<<endl;
    return res;
}

cv::Mat Frame::ComputeDepth(const int &cameraIdi, const int &cameraIdj, const float &u1, const float &v1, const float &u2, const float &v2)
{
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    mvpK[cameraIdi].copyTo(P1.rowRange(0,3).colRange(0,3));

    cv::Mat P2(3,4,CV_32F);
    cv::Mat tTcamji= cv::Mat::eye(4,4,CV_32F);

    for(int index = cameraIdi; index < cameraIdj; index++)
    {
        tTcamji = mvTcamji[index]* tTcamji;  //R trangCamId2,trangCamId1
    }

    cv::Mat R = tTcamji.rowRange(0,3).colRange(0,3);  //R = R
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    cv::Mat t = tTcamji.rowRange(0,3).col(3);
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = mvpK[cameraIdj]*P2;

    cv::Mat A(4,4,CV_32F);

    A.row(0) = u1*P1.row(2)-P1.row(0);
    A.row(1) = v1*P1.row(2)-P1.row(1);
    A.row(2) = u2*P2.row(2)-P2.row(0);
    A.row(3) = v2*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    cv::Mat x3D = vt.row(3).t();

    if(x3D.at<float>(3)==0)
        return cv::Mat();

    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
    if(cameraIdi!=0)
    {
        cv::Mat R0x, t0x;
        if(cameraIdi == 1)
        {
            cv::Mat T01 = mvTcamji[0].inv();
            R0x = T01.rowRange(0,3).colRange(0,3);  //R01
            t0x = T01.rowRange(0,3).col(3);
        }
        else if(cameraIdi == 2)
        {
            cv::Mat T02 = (mvTcamji[1]*mvTcamji[0]).inv();  //T02 = T01*T12
            R0x = T02.rowRange(0,3).colRange(0,3);
            t0x = T02.rowRange(0,3).col(3);
        }
        x3D = R0x*x3D+ t0x;
    }

    return x3D;

}


cv::Mat Frame::UnprojectStereo(const int &i)
{
    // KeyFrame::UnprojectStereo
    // mvDepth是在ComputeStereoMatches函数中求取的
    // mvDepth对应的校正前的特征点，可这里却是对校正后特征点反投影
    // KeyFrame::UnprojectStereo中是对校正前的特征点mvKeys反投影
    // 在ComputeStereoMatches函数中应该对校正后的特征点求深度？？ (wubo???)
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}


void Frame::CreateJointDescriptors(const vector<X3D> &vmp, const int &nonMonoNum)
{
    size_t matrix_size = vKeyPointsIndexMapPlus.size();
    mJointDescriptors.create(matrix_size, 32, CV_8UC1);

    for(int i = 0; i < matrix_size; i++)
    {
        if(i <= nonMonoNum-1)
        {
            vmp[i].mDescriptor.copyTo(mJointDescriptors.row(i));
        }
        else
        {
            auto it = std::find_if_not (vKeyPointsIndexMapPlus[i].begin(), vKeyPointsIndexMapPlus[i].end(), [](int i){if(i == -1) return true; else return false;} );
            int cameraId = it - vKeyPointsIndexMapPlus[i].begin();
            mvpDescriptors[cameraId].row(*it).copyTo(mJointDescriptors.row(i));
        }

    }
}

void Frame::ComputeBundledDepth(const vector<X3D> &vmp, const int &nonMonoNum)
{
    mvBDepth = vector<float>(N_L_R, -1.0f);

    for(int i = 0; i < N_L_R; i++)
    {
        if(i<=nonMonoNum-1)
        {
            mvBDepth[i] = vmp[i].mPos.at<float>(2);
        }

    }

}


cv::Mat Frame::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}


pair<bool, vector<float>> Frame::ComputeEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12)
{

    // 求出kp1在frame 2上对应的极线
    const float r = 2.0f*mvScaleFactors[kp2.octave];

    const float a = kp1.pt.x*F12.at<float>(0,0)+kp1.pt.y*F12.at<float>(1,0)+F12.at<float>(2,0);
    const float b = kp1.pt.x*F12.at<float>(0,1)+kp1.pt.y*F12.at<float>(1,1)+F12.at<float>(2,1);
    const float c = kp1.pt.x*F12.at<float>(0,2)+kp1.pt.y*F12.at<float>(1,2)+F12.at<float>(2,2);

    const float num = a*kp2.pt.x+b*kp2.pt.y+c;

    const float den = a*a+b*b;

    if(den==0)
        return {false, vector<float>()};

    const float dsqr = num*num/den;

    // 尺度越大，范围应该越大。
    // 金字塔最底层一个像素就占一个像素，在倒数第二层，一个像素等于最底层1.2个像素（假设金字塔尺度为1.2）
    return {dsqr<(3.84*mvLevelSigma2[kp2.octave]+r), {a,b,c}};

}

int Frame::BilinearInterpolation(const float &x, const float &y, const cv::Mat &image)
{

    int x0 = floor(x);
    int y0 = floor(y);
    int x1 = ceil(x);
    int y1 = ceil(y);
    int fxy;
    if(x0 == x1 && y0 == y1)
        fxy = image.at<uchar>(y, x);
    else
    {
        int fx0y0 = image.at<uchar>(y0, x0);
        int fx1y0 = image.at<uchar>(y0, x1);
        int fx0y1 = image.at<uchar>(y1, x0);
        int fx1y1 = image.at<uchar>(y1,x1);

        float fxy0 = (x1-x)*fx0y0/(x1-x0) + (x-x0)*fx1y0/(x1-x0);
        float fxy1 = (x1-x)*fx0y1/(x1-x0) + (x-x0)*fx1y1/(x1-x0);

        fxy = (y1-y)*fxy0/(y1-y0) + (y-y0)*fxy1/(y1-y0);
    }

    return fxy;
}

cv::Mat Frame::RemapSlidingWindow(const float &x, const float &y, const cv::Mat &image, const float &thita)
{
    cv::Mat res(11,11,CV_8UC1);

    for(int iy = 0;  iy<11; iy++)
    {
        for(int ix = 0; ix < 11; ix++)
        {
            float u = (x+ix-5)*cos(thita) - (y+iy-5)*sin(thita);
            float v = (x+ix-5)*sin(thita) + (y+iy-5)*cos(thita);
            //res.at<uchar>(iy,ix) = (uchar)BilinearInterpolation(x+ix-5,y+iy-5, image);
            res.at<uchar>(iy,ix) = (uchar)BilinearInterpolation(u,v, image);
        }
    }

    return res;
}

cv::Mat Frame::ComputeMapPointInWorldFrame(const cv::Mat &x3D)
{
    return mRwc*x3D+mOw;
}


} //namespace ORB_SLAM