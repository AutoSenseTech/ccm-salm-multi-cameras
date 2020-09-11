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

#ifndef CSLAM_MAPMATCHER_H_
#define CSLAM_MAPMATCHER_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <sstream>

//ROS
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Datatypes.h>
#include <cslam/Database.h>
#include <cslam/BundledKeyFramesDatabase.h>

#include <cslam/BundledMap.h>
#include <cslam/Map.h>
#include <cslam/KeyFrame.h>
#include <cslam/BundledKeyFrames.h>
#include <cslam/ORBVocabulary.h>
#include <cslam/ORBmatcher.h>
#include <cslam/Sim3Solver.h>
#include <cslam/Converter.h>
#include <cslam/Optimizer.h>
#include <cslam/BMapMerger.h>
#include <cslam/Communicator.h>

using namespace std;
using namespace estd;

namespace cslam{

//forward decs
class KeyFrameDatabase;
class BundledKeyFramesDatabase;
class BMapMerger;
class Map;
class BundledMap;
class Communicator;//zmf add
//-------------

// struct MapMatchHit
// {
// public:
//     typedef boost::shared_ptr<KeyFrame> kfptr;
//     typedef boost::shared_ptr<BundledKeyFrames> bkfptr;
//     typedef boost::shared_ptr<MapPoint> mpptr;
// public:
//     MapMatchHit(kfptr pKFCurr = nullptr, kfptr pKFMatch = nullptr, g2o::Sim3 g2oScw = g2o::Sim3(), std::vector<mpptr> vpLoopMapPoints = std::vector<mpptr>(), std::vector<mpptr> vpCurrentMatchedPoints = std::vector<mpptr>())
//         : mpKFCurr(pKFCurr), mpKFMatch(pKFMatch), mg2oScw(g2oScw),
//           mvpLoopMapPoints(vpLoopMapPoints), mvpCurrentMatchedPoints(vpCurrentMatchedPoints) {}
//     kfptr mpKFCurr;
//     kfptr mpKFMatch;
//     g2o::Sim3 mg2oScw;
//     std::vector<mpptr> mvpLoopMapPoints;
//     std::vector<mpptr> mvpCurrentMatchedPoints;
// };

struct MapMatchHit
{
public:
    typedef boost::shared_ptr<BundledKeyFrames> bkfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
public:
    MapMatchHit(bkfptr pBKFCurr = nullptr, bkfptr pBKFMatch = nullptr, g2o::Sim3 g2oScw = g2o::Sim3(), std::vector<mpptr> vpLoopMapPoints = std::vector<mpptr>(), std::vector<mpptr> vpCurrentMatchedPoints = std::vector<mpptr>())
        : mpBKFCurr(pBKFCurr), mpBKFMatch(pBKFMatch), mg2oScw(g2oScw),
          mvpLoopMapPoints(vpLoopMapPoints), mvpCurrentMatchedPoints(vpCurrentMatchedPoints) {}
    bkfptr mpBKFCurr;
    bkfptr mpBKFMatch;
    g2o::Sim3 mg2oScw;
    std::vector<mpptr> mvpLoopMapPoints;
    std::vector<mpptr> mvpCurrentMatchedPoints;
};

class MapMatcher : public boost::enable_shared_from_this<MapMatcher>
{
public:
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<BMapMerger> bmergeptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<BundledKeyFramesDatabase> bdbptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<BundledKeyFrames> bkfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<BundledMap> bmapptr;
	typedef boost::shared_ptr<Communicator> commptr; //zmf add

    //typedef pair<set<kfptr>,int> ConsistentGroup;
    typedef pair<set<bkfptr>,int> ConsistentGroup;

	commptr mpComm,mpComm1;
public:
    MapMatcher(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, dbptr pDB, vocptr pVoc, mapptr pMap0, mapptr pMap1, mapptr pMap2, mapptr pMap3);
    MapMatcher(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, bdbptr pBDB, vocptr pVoc, bmapptr pBMap0, bmapptr pBMap1, bmapptr pBMap2, bmapptr pBMap3);

    void Run();
	void SetCommunicator(commptr pComm) {mpComm=pComm;}//zmf add
    void SetCommunicator1(commptr pComm1) {mpComm1=pComm1;}//zmf add
    void InsertKF(kfptr pKF);
    void InsertBKF(bkfptr pBKF);
    void EraseKFs(vector<kfptr> vpKFs);
    int GetNumKFsinQueue();

private:
    bool CheckKfQueue();
    bool CheckBKfQueue(); //multi-camera
    bool DetectLoop();
    bool ComputeSim3();
    void CorrectLoop();
    void PublishLoopEdges();
    void ClearLoopEdges();

	kfptr FindClosestKeyFrame(const double &timeThreshold);
    vector<cv::Mat> TransferToCurrentFrame(map<double, vector<float>> inputPos, kfptr pclient0kf);
    map<double, vector<float>> TransferToOtherFrame(const vector<cv::Mat> &client0Pos);
    void PassSkelepos(vector<cv::Mat> pos);
    //Note: No need for a reset function for the MapMatcher

    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    ros::Publisher mPubMarker;

    visualization_msgs::Marker mMapMatchEdgeMsg;
    tf::TransformListener mTfListen;

	map<double, vector<float>> mmSkelepos;
    dbptr mpKFDB;
    bdbptr mpBKFDB;
    vocptr mpVoc;
    map<size_t,mapptr> mmpMaps;
    map<size_t,bmapptr> mmpBMaps;
    set<mapptr> mspMaps;
    set<bmapptr> mspBMaps;
    mapptr mpMap0;
    mapptr mpMap1;
    mapptr mpMap2;
    mapptr mpMap3;

    //multi-camera
    bmapptr mpBMap0;
    bmapptr mpBMap1;
    bmapptr mpBMap2;
    bmapptr mpBMap3;

    //mergeptr mpMapMerger;
    bmergeptr mpBMapMerger;
    std::list<kfptr> mlKfInQueue;
    std::list<bkfptr> mlBKfsInQueue;
    std::mutex mMutexKfInQueue;
    std::mutex mMutexBKfInQueue;
    // Loop detector parameters
    kfptr mpCurrentKF;
    bkfptr mpCurrentBKF;
    kfptr mpMatchedKF;
    bkfptr mpMatchedBKF;
    mapptr mpCurrMap;
    bmapptr mpCurrBMap;

    float mnCovisibilityConsistencyTh;
    std::map<bmapptr,std::vector<ConsistentGroup>> mmvConsistentGroups;

    std::vector<bkfptr> mvpEnoughConsistentCandidates;
    std::vector<kfptr> mvpCurrentConnectedKFs;
    std::vector<mpptr> mvpCurrentMatchedPoints;
    std::vector<mpptr> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;
    long unsigned int mLastLoopBKFid;

    cv::Mat mMatchMatrix;
    std::map<bmapptr,std::map<bmapptr,vector<MapMatchHit>>> mFoundMatches;

    // Fix scale in the stereo/RGB-D case - reamnant from ORB-SLAM, always set to false
    bool mbFixScale;
};

} //end namespace

#endif
