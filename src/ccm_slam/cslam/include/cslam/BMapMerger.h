#ifndef CSLAM_BMAPMERGER_H_
#define CSLAM_BMAPMERGER_H_

//C++
#include <boost/shared_ptr.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>

//ROS
#include <ros/ros.h>
#include <ros/publisher.h>

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Converter.h>
#include <cslam/Datatypes.h>
#include <cslam/CentralControl.h>

#include <cslam/BundledMap.h>
#include <cslam/BundledKeyFrames.h>
#include <cslam/Optimizer.h>
#include <cslam/MapMatcher.h>
#include <cslam/ClientHandler.h>
#include <cslam/MapPoint.h>

//THIRDPARTY
#include "thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

using namespace std;
using namespace estd;

namespace cslam{

//forward decs
class MapMatcher;
class ClientHandler;
class BundledMap;
class CentralControl;
class BundledKeyFrames;
class MapPoint;
struct MapMatchHit;
//-----------------

class BMapMerger
{
public:
    typedef boost::shared_ptr<BMapMerger> bmergeptr;
    typedef boost::shared_ptr<MapMatcher> matchptr;
    typedef boost::shared_ptr<ClientHandler> chptr;
    typedef boost::shared_ptr<BundledKeyFrames> bkfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<BundledMap> bmapptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
public:
    typedef pair<set<bkfptr>,int> ConsistentGroup;
    typedef map<bkfptr,g2o::Sim3,std::less<bkfptr>,
        Eigen::aligned_allocator<std::pair<const bkfptr, g2o::Sim3> > > BundledKeyFrameAndPose;
public:
    BMapMerger(matchptr pMatcher);
    bmapptr MergeBMaps(bmapptr pBMapCurr, bmapptr pBMapMatch, vector<MapMatchHit> vMatchHits);

    bool isBusy();
private:
    void SearchAndFuse(const BundledKeyFrameAndPose &CorrectedPosesBMap, std::vector<mpptr> vpLoopMapPoints);
    void SetBusy();
    void SetIdle();

    matchptr mpMatcher;

    bool bIsBusy;
    mutex mMutexBusy;

    std::vector<bkfptr> mvpCurrentConnectedBKFs;

    void RunGBA(idpair nLoopBKf, bmapptr pFusedBMap);
};

} //end namespace

#endif
