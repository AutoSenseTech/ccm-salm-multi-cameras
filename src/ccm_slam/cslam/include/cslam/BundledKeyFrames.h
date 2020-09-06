#ifndef CSLAM_BUNDLEDKEYFRAMES_H_
#define CSLAM_BUNDLEDKEYFRAMES_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <set>

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Datatypes.h>
#include <cslam/ORBVocabulary.h>
#include <cslam/Converter.h>
#include <cslam/MapPoint.h>
#include <cslam/BundledKeyFramesDatabase.h>


#include <cslam/BundledMap.h>
#include <cslam/Communicator.h>
#include <cslam/Frame.h>

//Thirdparty
#include <thirdparty/DBoW2/DBoW2/BowVector.h>
#include <thirdparty/DBoW2/DBoW2/FeatureVector.h>


// Msgs 
#include <ccmslam_msgs/BKF.h>
#include <ccmslam_msgs/BKFred.h>
#include <ccmslam_msgs/BMap.h>

using namespace std;
using namespace estd;

namespace cslam{
//forward decs
class Communicator;
class Frame;
class MapPoint;
class BundledMap;
class BundledKeyFramesDatabase;
//------------
class BundledKeyFrames: public boost::enable_shared_from_this<BundledKeyFrames>
{
public:
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<BundledKeyFrames> bkfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<BundledMap> bmapptr;
    typedef boost::shared_ptr<BundledKeyFramesDatabase> bdbptr;
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<CentralControl> ccptr;

public:
//---constructor---
    //BundledKeyFrames(Frame &F, const vector<kfptr> &vKeyFrames, bmapptr pBMap, bdbptr pBKFDB, commptr pComm, eSystemState SysState, size_t UniqueId);
    BundledKeyFrames(Frame &F, bmapptr pBMap, bdbptr pBKFDB, commptr pComm, eSystemState SysState, size_t UniqueId);
    BundledKeyFrames(ccmslam_msgs::BKF* pMsg, vocptr pVoc, bmapptr pBMap, bdbptr pBKFDB, commptr pComm, eSystemState SysState,
                       size_t UniqueId = defid, g2o::Sim3 mg2oS_wcurmap_wclientmap = g2o::Sim3()); 

    void EstablishInitialConnectionsClient();
//---communication---
    void ReduceMessage(ccmslam_msgs::BKF *pMsgFull, ccmslam_msgs::BKFred *pMsgRed);
    void ConvertToMessage(ccmslam_msgs::BMap &msgBMap, g2o::Sim3 mg2oS_wcurmap_wclientmap = g2o::Sim3(), bkfptr pRefBKfs = nullptr, bool bForceUpdateMsg = false);
    void UpdateFromMessage(ccmslam_msgs::BKF *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap = g2o::Sim3());
    void UpdateFromMessage(ccmslam_msgs::BKFred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap = g2o::Sim3());
    void WriteMembersFromMessage(ccmslam_msgs::BKF *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap);
    bool SetPoseFromMessage(ccmslam_msgs::BKF *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap);
    bool SetPoseFromMessage(ccmslam_msgs::BKFred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap);


    void SendMe();
    void MarkInOutBuffer() {unique_lock<mutex> lock(mMutexOut); mbInOutBuffer = true;}
    void UnMarkInOutBuffer() {unique_lock<mutex> lock(mMutexOut); mbInOutBuffer = false;}
    bool IsInOutBuffer() {unique_lock<mutex> lock(mMutexOut); return mbInOutBuffer;}
    bool CanBeForgotten();
    bool SentToClient(size_t ClientId){unique_lock<mutex> lock(mMutexOut); return msuSentToClient.count(ClientId);}
    void Ack(){unique_lock<mutex> lock(mMutexOut); mbAck = true;}
    bool AckSet(){unique_lock<mutex> lock(mMutexOut); return mbAck;}
    bool IsSent(){unique_lock<mutex> lock(mMutexOut); return mbSentOnce;}
    void SetSendFull();
    
//---set/get pointers---
    void AddCommPtr(commptr pComm){unique_lock<mutex> lockBMap(mMutexOut); mspComm.insert(pComm);}

//---visualization---
    bool mbFromServer;
    bool mbUpdatedByServer;

// Pose functions
    void SetPose(const cv::Mat &Tcw, bool bLock, bool bIgnorePoseMutex = false);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter(const int &cameraId);

// MapPoint observation functions
    void AddMapPoint(mpptr pMP, const size_t &index, bool bLock = false); 
    void EraseMapPointMatch(const size_t &index, bool bLock = false);
    void EraseMapPointMatch(mpptr pMP, bool bLock = false);
    void ReplaceMapPointMatch(const size_t &index, mpptr pMP, bool bLock = false, bool bOverrideLock = false);
    std::set<mpptr> GetMapPoints();
    std::vector<mpptr> GetMapPointMatches();
    mpptr GetMapPoint(const size_t &index);
    int TrackedMapPoints(const int &minObs);
// Bag of Words Representation
    void ComputeBoW(); 

// Covisibility graph functions
    void AddConnection(bkfptr pBKFs, const int &weight);
    void EraseConnection(bkfptr pBKFs);
    void UpdateConnections(bool bIgnoreMutex = false);
    void UpdateBestCovisibles();
    std::vector<bkfptr> GetVectorCovisibleBundledKeyFrames();
    std::vector<bkfptr> GetBestCovisibilityBundledKeyFrames(const int &N);
    std::vector<bkfptr> GetCovisiblesByWeight(const int &w);
    int GetWeight(bkfptr pBKFs);

// Spanning tree functions
    void AddChild(bkfptr pBKFs);
    void EraseChild(bkfptr pBKFs, bool bIgnoreMutex = false);
    void ChangeParent(bkfptr pBKFs);
    std::set<bkfptr> GetChilds();
    bkfptr GetParent(bool bIgnorePoseMutex = false);
    bool hasChild(bkfptr pBKFs);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int &cameraId) const;

    // Image
    bool IsInImage(const float &x, const float &y, const int &cameraId) const;

// Set/check bad / empty flag
    void SetBadFlag(bool bSuppressMapAction = false, bool bNoParent = false);
    bool isBad() {unique_lock<mutex> lock(mMutexConnections); return mbBad;}
    bool IsEmpty() {unique_lock<mutex> lock(mMutexConnections); return mbIsEmpty;}
    bool mbOmitSending;     

public:
    //Add new variable
    int cameraNum;
    //vector<kfptr> mvpKeyFrames;  //for multiplue camera

    vector<cv::Mat> mvTcamji;  //for multi camera
    cv::Mat mTi0;
    vector<cv::Mat> vmTi0;
    //vector<set<int>> vUpdatedKPIndex; //for multicamerastd::vector<pair<int, int>> vKeyPointsIndexMap;  //map for left feature index and right feature index
    vector<vector<int>> vKeyPointsIndexMapPlus;  //for multicamera
    //vector<set<int>> vUpdatedKPIndex; //for multicamera

    //---environment---
    double mdServerTimestamp;
    /*const*/ double mTimeStamp;
    double mdInsertStamp;

    //---IDs---
    static long unsigned int nNextId;
    idpair mFrameId;
    idpair mId;
    size_t mUniqueId;
    size_t mVisId;

    // Grid (to speed up feature matching)
    int mnGridCols;
    int mnGridRows;
    vector<float> mvGridElementWidthInv;
    vector<float> mvGridElementHeightInv;

    // Variables used by the tracking
    idpair mTrackReferenceForFrame;
    idpair mFuseTargetForBKFs;

    // Variables used by the local mapping
    idpair mBALocalForBKFs;
    idpair mBAFixedForBKFs;

    // Variables used by the keyframe database
    idpair mLoopQuery;
    idpair mMatchQuery;
    int mnLoopWords;
    float mLoopScore;
    idpair mRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    idpair mBAGlobalForBKFs;
    bool mbLoopCorrected;

    // Variables used by map merging
    idpair mCorrected_MM;

    // Calibration parameters
    vector<float> vfx, vfy, vcx, vcy, vinvfx, vinvfy;

    vector<int> mvpkeyPointsNum;

    // Number of KeyPoints fused
    int N;
    
    vector<vector<cv::KeyPoint>> mvKeysMultipleUn; //multi
    //for multi-camera descriptors
    vector<cv::Mat> mvpDescriptors;

    cv::Mat mDescriptors;
    vector<float> mvBDepth;
    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    std::vector<float> mvScaleFactors;
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;  //not multi

    vector<float> mvMinX;
    vector<float> mvMaxX;
    vector<float> mvMinY;
    vector<float> mvMaxY;
    vector<cv::Mat> mvpK;

    // Transformation to body frame (for KF write-out
    Eigen::Matrix4d mT_SC;

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:
    //---communication---
    bool mbInOutBuffer;
    bool mbSentOnce;
    bool mbSendFull;
    set<size_t> msuSentToClient;
    bool mbAck;


    //---infrastructure---
    bmapptr mpBMap;
    set<commptr> mspComm;
    eSystemState mSysState;
    bdbptr mpBundledKeyFramesDB;
    vocptr mpORBvocabulary;

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;
    vector<cv::Mat> vOw;

    bool mbPoseLock;
    bool mbPoseChanged;
    double mdPoseTime;
    cv::Mat Cw;

    // MapPoints associated to keypoints
    std::vector<mpptr> mvpMapPoints;
    std::vector<bool> mvbMapPointsLock;

    // Grid over the image to speed up feature matching
    void AssignFeaturesToGrid(const int &cameraId);
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY, const int &cameraId);
    std::vector< std::vector <std::vector<size_t> > > mGrid0;
    std::vector< std::vector <std::vector<size_t> > > mGrid1;
    std::vector< std::vector <std::vector<size_t> > > mGrid2;
    std::vector< std::vector <std::vector<size_t> > > mGrid3;

    map<bkfptr,int> mConnectedBundledKeyFramesWeights;
    vector<bkfptr> mvpOrderedConnectedBundledKeyFrames;
    vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    bkfptr mpParent;
    set<bkfptr> mspChildrens;
    set<bkfptr> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;
    bool mbIsEmpty;

    float mHalfBaseline; // Only for visualization

    //---mutexes---
    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    std::mutex mMutexOut;
    std::mutex mMapMutex;

};

}
#endif