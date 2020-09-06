#ifndef CSLAM_BUNDLEDMAP_H_
#define CSLAM_BUNDLEDMAP_H_
//C++
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <set>
#include <mutex>
#include <sstream>
#include <queue>
#include <ctime>

//CSLAM
#include <cslam/config.h>
#include <cslam/estd.h>
#include <cslam/Datatypes.h>
#include <cslam/CentralControl.h>
#include <cslam/MapPoint.h>
#include <cslam/BundledKeyFrames.h>
#include <cslam/Communicator.h>


//Msgs
#include <ccmslam_msgs/BMap.h>

using namespace std;
using namespace estd;


namespace cslam{
//forward decs
class CentralControl;
class Communicator;
class BundledKeyFrames;
class MapPoint;
//------------
class BundledMap: public boost::enable_shared_from_this<BundledMap>
{

public:
    typedef boost::shared_ptr<BundledMap> bmapptr;
    typedef boost::shared_ptr<BundledKeyFrames> bkfptr;
    typedef boost::shared_ptr<MapPoint> mpptr;
    typedef boost::shared_ptr<CentralControl> ccptr;
    typedef boost::shared_ptr<Communicator> commptr;

    struct bkftimecmp{
        bool operator() (const bkfptr pA, const bkfptr pB) const;
    };

    struct bkftimecmpsmaller{
        bool operator() (const bkfptr pA, const bkfptr pB) const;
    };

public:
//---Constructor---
    BundledMap(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, size_t BMapId, eSystemState SysState);
    BundledMap(const bmapptr &pBMapA, const bmapptr &pBMapB); //merge constructor

    

//---infrastructure---
    set<size_t> msuAssClients;
    void AddCCPtr(ccptr pCC);
    ccptr GetCCPtr(size_t nClientId);
    string mOdomFrame;
    size_t mBMapId;
    eSystemState mSysState;
    
//---Add/Erase data---
    void AddBundledKeyFrames(bkfptr pBKFs);
    void AddMapPoint(mpptr pMP);
    void EraseMapPoint(mpptr pMP);
    void EraseBundledKeyFrames(bkfptr pBKFs);
    void SetReferenceMapPoints(const std::vector<mpptr> &vpMPs);


//---Setter---
    void SetCommunicator(commptr pComm) {mspComm.insert(pComm);}

//---Getter---
    mpptr GetMpPtr(size_t MpId, size_t ClientId);
    mpptr GetMpPtr(idpair id){return GetMpPtr(id.first,id.second);} 
    bkfptr GetBKfsPtr(size_t BKfsId, size_t ClientId, bool bIgnoreMutex = false);    
    bkfptr GetBKfsPtr(idpair id){return GetBKfsPtr(id.first,id.second);}
    bkfptr GetErasedBKfsPtr(size_t BKfsId, size_t ClientId);
    bkfptr GetErasedBKfsPtr(idpair id){return GetErasedBKfsPtr(id.first,id.second);}
    vector<mpptr> GetAllMapPoints();
    long unsigned int MapPointsInMap();

    long unsigned  BundledKeyFramesInMap();
    bkfptr GetPredecessor(bkfptr pBKFs);


//---data---
    vector<bkfptr> mvpBundledKeyFramesOrigins;

//---map management---
    void MapTrimming(bkfptr pBKFcur);
    void FindLocalBKFsByTime(bkfptr pBKFcur,set<bkfptr>& sBKfsVicinity,priority_queue<int>& pqNativeBKFs,list<bkfptr>& lForeignBKFs, int nLocalBKFs);
    set<mpptr> mspMPsToErase;
    
//---mutexes & sync---
    bool LockBMapUpdate(){unique_lock<mutex> lock(mMutexBMapUpdate); if(!mbLockBMapUpdate){mbLockBMapUpdate = true; return true;} else return false;}
    bool LockPointCreation(){unique_lock<mutex> lock(mMutexPointCreation); if(!mbLockPointCreation){mbLockPointCreation = true; return true;} else return false;}
    void UnLockBMapUpdate(){unique_lock<mutex> lock(mMutexBMapUpdate);if(mbLockBMapUpdate){mbLockBMapUpdate = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"Map\": Attempt to UnLock BMapUpdate -- was not locked" << endl; throw estd::infrastructure_ex();}}
    void UnLockPointCreation(){unique_lock<mutex> lock(mMutexPointCreation);if(mbLockPointCreation){mbLockPointCreation = false;} else{cout << "\033[1;31m!!! ERROR !!!\033[0m \"Map\": Attempt to UnLock PointCreation -- was not locked" << endl; throw estd::infrastructure_ex();}}

    //BA
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

protected:
    //---infrastructure---
    ros::NodeHandle mNh;
    ros::NodeHandle mNhPrivate;
    
    std::set<ccptr> mspCC;
    set<commptr> mspComm; //comm ptrs of associated clients -- necessary to pass to MPs/KFs

    //---data---
    std::vector<mpptr> mvpReferenceMapPoints;
    long unsigned int mnMaxBKFsid;
    long unsigned int mnMaxMPid;
    long unsigned int mnMaxBKFsidUnique;
    long unsigned int mnMaxMPidUnique;
    long unsigned int mnLastBKfsIdUnique;

    std::map<idpair,mpptr> mmpMapPoints; //Performance: second container is not nice, but fast... //CHECKHERE   //就是我们原来的mspMapPoints ->addmappoint()
    std::map<idpair,bkfptr> mmpBundledKeyFrames; //Performance: second container is not nice, but fast... //CHECKHERE //就是我们原来的mspBundledKeyFrames ->addbkf()
    std::map<idpair,mpptr> mmpErasedMapPoints;
    std::map<idpair,bkfptr> mmpErasedBundledKeyFrames;

    bool mbOutdated;

    //---mutexes---
    std::mutex mMutexCC;
    std::mutex mMutexErased;
    std::mutex mMutexBMap;

    bool mbLockBMapUpdate;
    bool mbLockPointCreation;
    std::mutex mMutexBMapUpdate;
    std::mutex mMutexPointCreation; // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexOutdated;
    std::mutex mMutexStopGBAInProgess;

    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbNoStartGBA;
    bool mbStopGBAInProgress;
    #ifdef DONOTINTERRUPTMERGE
    bool mbMergeStepGBA;
    #endif

    #ifdef FINALBA
    bool mbGBAinterrupted;
    #endif

};


}
#endif