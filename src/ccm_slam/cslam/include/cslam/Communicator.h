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

#ifndef CSLAM_COMMUNICATORV2_H_
#define CSLAM_COMMUNICATORV2_H_

//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/bind.hpp>
#include <mutex>
#include <sstream>
#include <algorithm>

#include <time.h>

//ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Datatypes.h>
#include <cslam/CentralControl.h>
#include <cslam/ORBVocabulary.h>
#include <cslam/KeyFrame.h>
#include <cslam/BundledKeyFrames.h>
#include <cslam/MapPoint.h>
#include <cslam/BundledMap.h>
#include <cslam/Map.h>
#include <cslam/Database.h>
#include <cslam/BundledKeyFramesDatabase.h>
#include <cslam/MapMatcher.h>
#include <cslam/Mapping.h>

//Msgs
#include <ccmslam_msgs/Map.h>
#include <ccmslam_msgs/BMap.h>
#include<ccmslam_msgs/ObjectPosition.h>

using namespace std;
using namespace estd;

namespace cslam
{

//forward decs
class MapMatcher;
class Map;
class BundledMap;
class LocalMapping;
class MapMatcher;
class CentralControl;
class KeyFrameDatabase;
class BundledKeyFramesDatabase;
class KeyFrame;
class BundledKeyFrames;
class MapPoint;
class Skeleton;//zmfadd
//------------------

class Communicator : public boost::enable_shared_from_this<Communicator>
{
public:
    typedef boost::shared_ptr<Communicator> commptr;
    typedef boost::shared_ptr<KeyFrame> kfptr;
    typedef boost::shared_ptr<BundledKeyFrames> bkfptr;
	typedef boost::shared_ptr<Skeleton> skptr;//zmf add
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<Map> mapptr;
    typedef boost::shared_ptr<BundledMap> bmapptr;
    typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
    typedef boost::shared_ptr<BundledKeyFramesDatabase> bdbptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<LocalMapping> mappingptr;

    typedef pair<size_t,kfptr> AckPairKF;
    typedef pair<size_t,bkfptr> AckPairBKF;
    typedef pair<size_t,mpptr> AckPairMP;

    typedef pair<ccmslam_msgs::KF,ccmslam_msgs::KFred> msgKFPair;
    typedef pair<ccmslam_msgs::BKF,ccmslam_msgs::BKFred> msgBKFPair;
    typedef pair<ccmslam_msgs::MP,ccmslam_msgs::MPred> msgMPPair;

    //---additional functions
    struct kfcmp{
        bool operator() (const kfptr pA, const kfptr pB) const;
    };
    //for multi-camare
    struct bkfcmp{
        bool operator() (const bkfptr pA, const bkfptr pB) const;
    };

    struct mpcmp{
        bool operator() (const mpptr pA, const mpptr pB) const;
    };

public:
    //---constructor---
    Communicator(ccptr pCC, vocptr pVoc, bmapptr pBMap, bdbptr pBKFDB);

    //---main---
    void RunClient();
    void RunServer();

    //---getter/setter---
    void SetMapping(mappingptr pMapping) {mpMapping = pMapping;}
    void ChangeMap(mapptr pMap){mpMap = pMap;}
    void ChangeBMap(bmapptr pBMap){mpBMap = pBMap;}
    void SetMapMatcher(matchptr pMatch) {mpMapMatcher = pMatch;}
    dbptr GetDbPtr(){return mpDatabase;}
    size_t GetClientId(){return mClientId;}
    idpair GetNearestKFid(){return mNearestKfId;}
    idpair GetNearestBKFsid(){return mNearestBKfsId;}

    //---callbacks---
    void MapCbClient(ccmslam_msgs::BMapConstPtr pMsg);
    void MapCbServer(ccmslam_msgs::MapConstPtr pMsg);
    void RequestReset();
    void RequestResetExternal();

    //--- data transfer---
    void PassKftoComm(kfptr pKf);
    void PassBKfstoComm(bkfptr pBKfs);

    void PassMptoComm(mpptr pMp);
	void PassSkPostoComm(skptr skele);
    void PassSkePointstoComm(map<double, vector<float>> &result);

    map<double,vector<float>> GetSkelepos();//这个函数是负责将comm 存储的骨架信息 传给 mapmatch 
    void DeleteMpFromBuffer(mpptr pMP);

public:
    //---infrastructure---
    ccptr mpCC;
    mapptr mpMap;
    bmapptr mpBMap;
    dbptr mpDatabase;
    bdbptr mpBundledKeyFramesDatabase;
    vocptr mpVoc;
    mappingptr mpMapping;
    matchptr mpMapMatcher;
    size_t mClientId;
    const double mdPeriodicTime;

    kfptr mpNearestKF; //client: store last nearest KF -- server: store current nearest KF
    bkfptr mpNearestBKFs; //client: store last nearest KF -- server: store current nearest KF
    idpair mNearestKfId;
    idpair mNearestBKfsId;

	//zmf add 
    map<double,vector<float>> skelePos_list;//client 端 定义的存储骨架信息的数据结构
    map<double,vector<float>> skelePos_list_server;////server 端 定义的存储骨架信息的数据结构

    map<double,vector<float>> skelePos_list_receive_from_mapmatch;
    int mKfItBound;
    int mBKfsItBound;
    int mMpItBound;
    int mKfItBoundPub;
    int mBKfsItBoundPub;
    int mMpItBoundPub;
    int mMaxMsgSentPerIt;

    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;

    ros::Publisher mPubMap,mPubSke;
    ros::Subscriber mSubMap,mSubSke;

    ros::Publisher mPubBMap;
    ros::Subscriber mSubBMap;

    string mSubKfTopicName;
    string mSubMpTopicName;
    int mPubMapBufferSize, mSubMapBufferSize;

    //---buffer checks---
    bool CheckBufferKfIn();
    bool CheckBufferKfOut();
    bool CheckBufferMpIn();
    bool CheckBufferMpOut();

    //---publish/receive---
	
	
	void PublishObjectPositionServer();//zmf add
    void PublishObjectPositionClient();//zmf add
    void serversave_skelepos(ccmslam_msgs::ObjectPositionConstPtr pMsg);//负责接收终端0发过来的骨架信息
    void PublishMapServer();
    void PublishMapClient();
    double mdLastTimePub;
    void ProcessKfInServer();
    void ProcessKfInClient();
    void ProcessBKfsInClient();

    void ProcessMpInServer();
    void ProcessMpInClient();
    size_t mMsgCountLastMapMsg;
    kfptr mpKFLastFront;
    bkfptr mpBKFsLastFront;
    size_t mnMaxKfIdSent;
    size_t mnMaxBKfsIdSent;

    //---IO buffers---
    set<kfptr,kfcmp> mspBufferKfOut;
    set<bkfptr,bkfcmp> mspBufferBKfsOut;
    set<mpptr,mpcmp> mspBufferMpOut;
    list<msgKFPair> mlBufKFin;
    list<msgBKFPair> mlBufBKFsin;
    list<msgMPPair> mlBufMPin;

    list<kfptr> mlpAddedKfs;
    list<bkfptr> mlpAddedBKfs;

    set<size_t> msAcksKF;
    set<size_t> msAcksMP;
    void SetWeakAckKF(size_t id);
    void SetWeakAckMP(size_t id);
    size_t mnWeakAckKF;
    size_t mnWeakAckBKFs;
    size_t mnWeakAckMP;
    list<AckPairKF> mlKfOpenAcks;
    list<AckPairBKF> mlBKfsOpenAcks;
    list<AckPairMP> mlMpOpenAcks;

    //---Reset---
    void ResetIfRequested();
    void ResetCommunicator();
    void ResetDatabase();
    void ResetMap();
    void ResetMapping();
    void ResetMapMatcher();
    bool mbResetRequested;

    //--mutexes
    mutex mMutexBuffersOut;
    mutex mMutexBuffersIn;
    mutex mMutexMsgBuffer;
    mutex mMutexReset;
    mutex mMutexLastMsgId;
    mutex mMutexNearestKf;

    //--Final BA if interrupted
    size_t mnEmptyMsgs;

    //monitoring //CHECKHERE
    size_t mOutMapCount;
    size_t mServerMapCount;
    void CheckOrdering(ccmslam_msgs::Map msgMap);
    void ShowBufferContent(bool bGetMutexBufferIn = false, bool bGetMutexBufferOut = false);
};

} //end ns

#endif
