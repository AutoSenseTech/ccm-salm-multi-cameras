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
    void UpdateAssociatedData(); //need to be called after merge constructor. Cannot be called by merge constructor because of usage of shared_from_this()
    BundledMap& operator=(BundledMap& rhs);
    

//---infrastructure---
    set<size_t> msuAssClients;
    void AddCCPtr(ccptr pCC);
    set<ccptr> GetCCPtrs();
    ccptr GetCCPtr(size_t nClientId);
    void SetOutdated(){unique_lock<mutex> lock(mMutexOutdated); mbOutdated=true;}
    bool GetOutdated(){unique_lock<mutex> lock(mMutexOutdated); return mbOutdated;}
    string mOdomFrame;
    size_t mBMapId;
    eSystemState mSysState;

//---Add/Erase data---
    void AddBundledKeyFrames(bkfptr pBKFs);
    void AddMapPoint(mpptr pMP);
    void EraseMapPoint(mpptr pMP);
    void EraseBundledKeyFrames(bkfptr pBKFs);
    void SetReferenceMapPoints(const std::vector<mpptr> &vpMPs);
    void ClearBadMPs();

//---Setter---
    void SetCommunicator(commptr pComm) {mspComm.insert(pComm);}

//---Getter---
    bkfptr GetBKfsPtr(size_t BKfsId, size_t ClientId, bool bIgnoreMutex = false);    
    bkfptr GetBKfsPtr(idpair id){return GetBKfsPtr(id.first,id.second);}
    bkfptr GetRandBKfPtr();
    mpptr GetMpPtr(size_t MpId, size_t ClientId);
    mpptr GetMpPtr(idpair id){return GetMpPtr(id.first,id.second);} 
    bkfptr GetErasedBKfsPtr(size_t BKfsId, size_t ClientId);
    bkfptr GetErasedBKfsPtr(idpair id){return GetErasedBKfsPtr(id.first,id.second);}

    vector<bkfptr> GetAllBundledKeyFrames();
    vector<mpptr> GetAllMapPoints();
    long unsigned int MapPointsInMap();
    long unsigned  BundledKeyFramesInMap();

    long unsigned int GetMaxBKFid();
    long unsigned int GetMaxMPid();
    long unsigned int GetMaxBKFidUnique();
    long unsigned int GetMaxMPidUnique();
    long unsigned int GetLastBKfIdUnique();

    bkfptr GetPredecessor(bkfptr pBKFs);

    std::vector<mpptr> GetMvpReferenceMapPoints() {return mvpReferenceMapPoints;}
    std::map<idpair,mpptr> GetMmpMapPoints() {return mmpMapPoints;}
    std::map<idpair,bkfptr> GetMmpBundledKeyFrames() {return mmpBundledKeyFrames;}
    std::map<idpair,mpptr> GetMmpErasedMapPoints() {return mmpErasedMapPoints;}
    std::map<idpair,bkfptr> GetMmpErasedBundledKeyFrames() {return mmpErasedBundledKeyFrames;}
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

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    void setRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbRunningGBA = true;
    }
    void unsetRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbRunningGBA = false;
    }

    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }
    void setFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbFinishedGBA = true;
    }
    void unsetFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbFinishedGBA = false;
    }

    bool isNoStartGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbNoStartGBA;
    }
    void setNoStartGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbNoStartGBA = true;
    }
    void unsetNoStartGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbNoStartGBA = false;
    }

    #ifdef DONOTINTERRUPTMERGE
    bool isMergeStepGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbMergeStepGBA;
    }
    void setMergeStepGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbMergeStepGBA = true;
    }
    void unsetMergeStepGBA(){
//        unique_lock<std::mutex> lock(mMutexGBA); //already locked when this is used
        mbMergeStepGBA = false;
    }
    #endif

    void StopGBA();

    #ifdef FINALBA
    bool isGBAinterrupted(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbGBAinterrupted;
    }
    void setGBAinterrupted(){
        unique_lock<std::mutex> lock(mMutexGBA);
        mbGBAinterrupted = true;
    }
    void unsetGBAinterrupted(){
//        unique_lock<std::mutex> lock(mMutexGBA); //already locked when this is used
        mbGBAinterrupted = false;
    }
    #endif

    void RequestBA(size_t nClientId);
    void RunGBA(idpair nLoopBKF);
    set<size_t> msnFinishedAgents;


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