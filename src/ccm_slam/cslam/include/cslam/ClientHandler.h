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


#ifndef CSLAM_CLIENTHANDLER_H_
#define CSLAM_CLIENTHANDLER_H_

//C++
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <mutex>
#include <sstream>

//ROS
//...

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Datatypes.h>
#include <cslam/CentralControl.h>
#include <cslam/ORBVocabulary.h>
#include <cslam/BundledMap.h>
#include <cslam/Map.h>
#include <cslam/Database.h>
#include <cslam/BundledKeyFramesDatabase.h>
#include <cslam/Mapping.h>
#include <cslam/Communicator.h>
#include <cslam/MapMatcher.h>
#include <cslam/Communicator.h>
#include <cslam/LoopFinder.h>
#include <cslam/Viewer.h>
#include <cslam/Tracking.h>
#include <cslam/skeleton.h>
//msg
#include<ccmslam_msgs/ObjectPosition.h>

//Thirdparty
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
using namespace estd;

namespace cslam{

//forward decs
class Tracking;
class Viewer;
class LoopFinder;
struct SystemEndpoint;
class Skeleton;//zmfadd
//----------------


class ClientHandler : public boost::enable_shared_from_this<ClientHandler>
{
public:
    typedef boost::shared_ptr<Tracking> trackptr;
    typedef boost::shared_ptr<Viewer> viewptr;
    typedef boost::shared_ptr<LocalMapping> mappingptr;
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<LoopFinder> lfptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<BundledMap> bmapptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<BundledKeyFramesDatabase> bdbptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<BundledKeyFrames> bkfptr;
	typedef boost::shared_ptr<Skeleton> skptr;//zmf add
public:
    ClientHandler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, vocptr pVoc, dbptr pDB, mapptr pMap, size_t ClientId, uidptr pUID, eSystemState SysState, const string &strCamFile, viewptr pViewer);
    ClientHandler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, vocptr pVoc, bdbptr pBDB, bmapptr pBMap, size_t ClientId, uidptr pUID, eSystemState SysState, const string &strCamFile, viewptr pViewer);

    #ifdef LOGGING
    void InitializeThreads(boost::shared_ptr<estd::mylog> pLogger = nullptr);
    #else
    void InitializeThreads();
    #endif

    //---getter/setter---
    void SetMapMatcher(matchptr pMatch);
    void ChangeMap(mapptr pMap, g2o::Sim3 g2oS_wnewmap_wcurmap);
    void ChangeBMap(bmapptr pBMap, g2o::Sim3 g2oS_wnewmap_wcurmap);
    commptr GetCommPtr(){return mpComm;}
    trackptr GetTrackPtr(){return mpTracking;}
    mappingptr GetMappingPtr(){return mpMapping;}
    dbptr GetDbPtr(){return mpKFDB;}
    vocptr GetVocPtr(){return mpVoc;}
    kfptr GetCurrentRefKFfromTracking();
    bkfptr GetCurrentRefBKFsfromTracking();
    int GetNumKFsinLoopFinder();
    int GetNumKFsinMapMatcher();

    //---forwarding---
    void ClearCovGraph(size_t MapId);

    //---Agent side---
	void ObejectPositionCb(ccmslam_msgs::ObjectPositionConstPtr pMsg);
	
    //void CamImgCb(sensor_msgs::ImageConstPtr msgLeft, sensor_msgs::ImageConstPtr msgRight,sensor_msgs::ImageConstPtr msgdepth_img);//这里需要修改
    //void CamImgCb(sensor_msgs::ImageConstPtr msg0, sensor_msgs::ImageConstPtr msg1,sensor_msgs::ImageConstPtr msgdepth_img);
    //void CamImgCb(sensor_msgs::ImageConstPtr msg0, sensor_msgs::ImageConstPtr msg1, sensor_msgs::ImageConstPtr msg2, sensor_msgs::ImageConstPtr msg3, sensor_msgs::ImageConstPtr msgdepth_img);//这里需要修改

    void CamImgCb(sensor_msgs::ImageConstPtr msg0, sensor_msgs::ImageConstPtr msg1);
    void Reset();
    // <hxy>测试样例
    void hello(sensor_msgs::ImageConstPtr pMsg, sensor_msgs::ImageConstPtr pMsg2);
    //  </hxy>


//    #ifdef LOGGING
//    void SetLogger(boost::shared_ptr<estd::mylog> pLogger);
//    #endif

// private:
public:
    #ifdef LOGGING
    void InitializeCC(boost::shared_ptr<estd::mylog> pLogger);
    #else
    void InitializeCC();
    #endif
    void InitializeClient();
    void InitializeServer();
	//////////////////zmf add
    double OPTimeStamp;
    string Client_id;
    std::vector<vector<float>> ObjectPostition;//个三维点坐标
    //////////////////zmf add

    //infrastructure
    ccptr mpCC;
    mapptr mpMap;
    dbptr mpKFDB;
    bmapptr mpBMap;
    bdbptr mpBKFDB;

    vocptr mpVoc;
    commptr mpComm;
    mappingptr mpMapping;
    viewptr mpViewer;
    //agent only
    trackptr mpTracking;
	skptr mpSkeleton;//zmf add
    //server only
    lfptr mpLoopFinder;
    matchptr mpMapMatcher;
    uidptr mpUID;
    eSystemState mSysState;

    const string mstrCamFile;

    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;
    ros::Subscriber mSubCam;
	ros::Subscriber mSubOP;

    //threads
    threadptr mptMapping;
    threadptr mptComm;
    threadptr mptLoopClosure;
    threadptr mptViewer;
	threadptr mptskeleton;

    //data
    size_t mClientId;
    g2o::Sim3 mg2oS_wcurmap_wclientmap; //transformation from map into client

    //reset
    bool mbReset;

    //mutexes
    mutex mMutexThreads;
    mutex mMutexReset;
};

} //end ns

#endif
