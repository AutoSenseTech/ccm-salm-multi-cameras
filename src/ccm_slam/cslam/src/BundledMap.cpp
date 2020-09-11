#include <cslam/BundledMap.h>

namespace cslam {

BundledMap::BundledMap(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, size_t BMapId, eSystemState SysState)
    :  mnMaxMPid(0),
      mnMaxBKFsidUnique(0),mnMaxMPidUnique(0),
      mNh(Nh), mNhPrivate(NhPrivate),
      mBMapId(BMapId),mbOutdated(false),
      mSysState(SysState),
      mbLockBMapUpdate(false),mbLockPointCreation(false)
      ,mnLastBKfsIdUnique(0)
      ,mbStopGBA(false),mbRunningGBA(false),mbFinishedGBA(false),
    #ifdef FINALBA
    mbGBAinterrupted(false),
    #endif
    mbNoStartGBA(false),mbStopGBAInProgress(false)
    #ifdef DONOTINTERRUPTMERGE
    ,mbMergeStepGBA(false)
    #endif
{
    uint myseed = time(NULL);
    srand (myseed);
    std::cout << "BundledMap " << mBMapId << " rand seed: " << myseed << std::endl;

    string SysType;
    if(mSysState == eSystemState::CLIENT)
    {
        SysType = "Client";
    }
    else if(mSysState == eSystemState::SERVER)
    {
        SysType = "Server";
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Map::Map(): invalid systems state: " << mSysState << endl;
        throw infrastructure_ex();
    }

    cout << "+++++ BMap " << mBMapId << " Initialized +++++" << endl;
}

BundledMap::BundledMap(const bmapptr &pBMapTarget, const bmapptr &pBMapToFuse)
    : mNh(pBMapTarget->mNh),mNhPrivate(pBMapTarget->mNhPrivate),
      mBMapId(pBMapTarget->mBMapId),mbOutdated(false),
      mbLockBMapUpdate(false),mbLockPointCreation(false)
      ,mnLastBKfsIdUnique(pBMapToFuse->GetLastBKfIdUnique())
    ,mbStopGBA(false),mbRunningGBA(false),mbFinishedGBA(false),
    #ifdef FINALBA
    mbGBAinterrupted(false),
    #endif
    mbNoStartGBA(false),mbStopGBAInProgress(false)
    #ifdef DONOTINTERRUPTMERGE
    ,mbMergeStepGBA(false)
    #endif
{

    mSysState = pBMapTarget->mSysState;

    string SysType;
    if(mSysState == eSystemState::CLIENT)
    {
        SysType = "Client";
    }
    else if(mSysState == eSystemState::SERVER)
    {
        SysType = "Server";
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Map::Map(): invalid systems state: " << mSysState << endl;
        throw infrastructure_ex();
    }

    //data Map A
    set<size_t> msuAssClientsA = pBMapTarget->msuAssClients;
    set<size_t> msnFinishedAgentsA = pBMapTarget->msnFinishedAgents;
    vector<bkfptr> mvpBundledKeyFrameOriginsA = pBMapTarget->mvpBundledKeyFramesOrigins;
    long unsigned int mnMaxBKFidA = pBMapTarget->GetMaxBKFid();
    long unsigned int mnMaxMPidA = pBMapTarget->GetMaxMPid();
    long unsigned int mnMaxBKFidUniqueA = pBMapTarget->GetMaxBKFidUnique();
    long unsigned int mnMaxMPidUniqueA = pBMapTarget->GetMaxMPidUnique();
    std::map<idpair,mpptr> mmpMapPointsA = pBMapTarget->GetMmpMapPoints();
    std::map<idpair,bkfptr> mmpBundledKeyFramesA = pBMapTarget->GetMmpBundledKeyFrames();
    std::map<idpair,mpptr> mmpErasedMapPointsA = pBMapTarget->GetMmpErasedMapPoints();
    std::map<idpair,bkfptr> mmpErasedBundledKeyFramesA = pBMapTarget->GetMmpErasedBundledKeyFrames();
    set<ccptr> spCCA = pBMapTarget->GetCCPtrs();

    //data Map B
    set<size_t> msuAssClientsB = pBMapToFuse->msuAssClients;
    set<size_t> msnFinishedAgentsB = pBMapToFuse->msnFinishedAgents;
    vector<bkfptr> mvpBundledKeyFrameOriginsB = pBMapToFuse->mvpBundledKeyFramesOrigins;
    long unsigned int mnMaxBKFidB = pBMapToFuse->GetMaxBKFid();
    long unsigned int mnMaxMPidB = pBMapToFuse->GetMaxMPid();
    long unsigned int mnMaxBKFidUniqueB = pBMapToFuse->GetMaxBKFidUnique();
    long unsigned int mnMaxMPidUniqueB = pBMapToFuse->GetMaxMPidUnique();
    std::map<idpair,mpptr> mmpMapPointsB = pBMapToFuse->GetMmpMapPoints();
    std::map<idpair,bkfptr> mmpBundledKeyFramesB = pBMapToFuse->GetMmpBundledKeyFrames();
    std::map<idpair,mpptr> mmpErasedMapPointsB = pBMapToFuse->GetMmpErasedMapPoints();
    std::map<idpair,bkfptr> mmpErasedBundledKeyFramesB = pBMapToFuse->GetMmpErasedBundledKeyFrames();
    set<ccptr> spCCB = pBMapToFuse->GetCCPtrs();

    //fill new map
    mOdomFrame = pBMapTarget->mOdomFrame;

    msuAssClients.insert(msuAssClientsA.begin(),msuAssClientsA.end());
    msuAssClients.insert(msuAssClientsB.begin(),msuAssClientsB.end());
    msnFinishedAgents.insert(msnFinishedAgentsA.begin(),msnFinishedAgentsA.end());
    msnFinishedAgents.insert(msnFinishedAgentsB.begin(),msnFinishedAgentsB.end());
    mvpBundledKeyFramesOrigins.insert(mvpBundledKeyFramesOrigins.end(),mvpBundledKeyFrameOriginsA.begin(),mvpBundledKeyFrameOriginsA.end());
    mvpBundledKeyFramesOrigins.insert(mvpBundledKeyFramesOrigins.end(),mvpBundledKeyFrameOriginsB.begin(),mvpBundledKeyFrameOriginsB.end());
    mnMaxBKFsid = std::max(mnMaxBKFidA,mnMaxBKFidB);
    mnMaxMPid = std::max(mnMaxMPidA,mnMaxMPidB);
    mnMaxBKFsidUnique = std::max(mnMaxBKFidUniqueA,mnMaxBKFidUniqueB);
    mnMaxMPidUnique = std::max(mnMaxMPidUniqueA,mnMaxMPidUniqueB);
    mmpMapPoints.insert(mmpMapPointsA.begin(),mmpMapPointsA.end());
    mmpMapPoints.insert(mmpMapPointsB.begin(),mmpMapPointsB.end());
    mmpBundledKeyFrames.insert(mmpBundledKeyFramesA.begin(),mmpBundledKeyFramesA.end());
    mmpBundledKeyFrames.insert(mmpBundledKeyFramesB.begin(),mmpBundledKeyFramesB.end());
    mmpErasedMapPoints.insert(mmpErasedMapPointsA.begin(),mmpErasedMapPointsA.end());
    mmpErasedMapPoints.insert(mmpErasedMapPointsB.begin(),mmpErasedMapPointsB.end());
    mmpErasedBundledKeyFrames.insert(mmpErasedBundledKeyFramesA.begin(),mmpErasedBundledKeyFramesA.end());
    mmpErasedBundledKeyFrames.insert(mmpErasedBundledKeyFramesB.begin(),mmpErasedBundledKeyFramesB.end());
    mspCC.insert(spCCA.begin(),spCCA.end());
    mspCC.insert(spCCB.begin(),spCCB.end());

    #ifdef FINALBA
    if(pBMapTarget->isGBAinterrupted() || pBMapToFuse->isGBAinterrupted())
        this->mbGBAinterrupted = true;
    #endif

    for(set<ccptr>::const_iterator sit = mspCC.begin();sit!=mspCC.end();++sit)
    {
        ccptr pCC = *sit;
        mspComm.insert(pCC->mpCH->GetCommPtr());
    }

    for(set<size_t>::iterator sit = msnFinishedAgentsA.begin();sit!=msnFinishedAgentsA.end();++sit)
        cout << "Target BMap Finished Agents: " << *sit << endl;

    for(set<size_t>::iterator sit = msnFinishedAgents.begin();sit!=msnFinishedAgents.end();++sit)
        cout << "Merged BMap Finished Agents: " << *sit << endl;

    //----------------------------
}

void BundledMap::UpdateAssociatedData()
{

    //replace associated maps
    for(std::map<idpair,bkfptr>::iterator mit = mmpBundledKeyFrames.begin();mit!=mmpBundledKeyFrames.end();++mit)
    {
        bkfptr pBKF = mit->second;
        pBKF->ReplaceBMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pBKF->AddCommPtr(pComm);
        }
    }

    for(std::map<idpair,mpptr>::iterator mit = mmpMapPoints.begin();mit!=mmpMapPoints.end();++mit)
    {
        mpptr pMP = mit->second;
        pMP->ReplaceBMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pMP->AddCommPtr(pComm);
        }
    }

    for(map<idpair,bkfptr>::iterator mit = mmpErasedBundledKeyFrames.begin();mit!=mmpErasedBundledKeyFrames.end();++mit)
    {
        bkfptr pBKF = mit->second;
        pBKF->ReplaceBMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pBKF->AddCommPtr(pComm);
        }
    }

    for(map<idpair,mpptr>::iterator mit = mmpErasedMapPoints.begin();mit!=mmpErasedMapPoints.end();++mit)
    {
        mpptr pMP = mit->second;
        pMP->ReplaceBMap(this->shared_from_this());
        for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
        {
            commptr pComm = *sit2;
            pMP->AddCommPtr(pComm);
        }
    }
}

void BundledMap::AddBundledKeyFrames(bkfptr pBKFs)
{

    unique_lock<mutex> lock(mMutexBMap);
    if(mSysState == eSystemState::CLIENT)
    {
        std::map<idpair,bkfptr>::iterator mit = mmpBundledKeyFrames.find(pBKFs->mId);
        if(mit != mmpBundledKeyFrames.end())
        {
            return;
        }
        else
        {
            commptr pComm = *(mspComm.begin());
            if(!pBKFs->mbFromServer)
                pComm->PassBKfstoComm(pBKFs);
        }
    }
    else if(mSysState == eSystemState::SERVER)
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            pBKFs->AddCommPtr(*sit);
        }

        mnLastBKfsIdUnique = pBKFs->mUniqueId;
    }

    if(pBKFs->mId.first>mnMaxBKFsid)
        mnMaxBKFsid=pBKFs->mId.first;
    if(pBKFs->mUniqueId>mnMaxBKFsidUnique)
        mnMaxBKFsidUnique=pBKFs->mUniqueId;

    mmpBundledKeyFrames[pBKFs->mId] = pBKFs;

    if(mSysState == SERVER)
        if(mmpBundledKeyFrames.size() % 50 == 0)
            cout << "BKFs in BMap: " << mmpBundledKeyFrames.size() << endl;

}

void BundledMap::AddMapPoint(mpptr pMP)
{
    unique_lock<mutex> lock(mMutexBMap);

    if(mSysState == eSystemState::CLIENT)
    {
        std::map<idpair,mpptr>::iterator mit = mmpMapPoints.find(pMP->mId);
        if(mit != mmpMapPoints.end())
        {
            return;
        }
        else
        {
            commptr pComm = *(mspComm.begin());
            if(!pMP->mbFromServer)
            {
                pComm->PassMptoComm(pMP);
            }
        }
    }
    else if(mSysState == eSystemState::SERVER)
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            pMP->AddCommPtr(*sit);
        }
    }

    if(pMP->mId.first>mnMaxMPid)
        mnMaxMPid=pMP->mId.first;
    if(pMP->mUniqueId>mnMaxMPidUnique)
        mnMaxMPidUnique=pMP->mUniqueId;

    mmpMapPoints[pMP->mId] = pMP;
}

void BundledMap::EraseMapPoint(mpptr pMP)
{
    unique_lock<mutex> lock(mMutexBMap);

    std::map<idpair,mpptr>::iterator mit = mmpMapPoints.find(pMP->mId);
    if(mit != mmpMapPoints.end()) mmpMapPoints.erase(mit);

    if(msuAssClients.count(pMP->mId.second))
    {
        unique_lock<mutex> lock2(mMutexErased);
        mmpErasedMapPoints[pMP->mId] = pMP;
    }
}


void BundledMap::EraseBundledKeyFrames(bkfptr pBKFs)
{
    if(pBKFs->mId.first == 0)
    {
        cout << COUTFATAL << " cannot erase Origin-BKFs" << endl;
        throw infrastructure_ex();
    }

    unique_lock<mutex> lock(mMutexBMap);

    std::map<idpair,bkfptr>::iterator mit = mmpBundledKeyFrames.find(pBKFs->mId);
    if(mit != mmpBundledKeyFrames.end()) mmpBundledKeyFrames.erase(mit);

    if(msuAssClients.count(pBKFs->mId.second))
    {
        unique_lock<mutex> lock2(mMutexErased);
        mmpErasedBundledKeyFrames[pBKFs->mId] = pBKFs;
    }
}

void BundledMap::SetReferenceMapPoints(const vector<mpptr> &vpMPs)
{
    unique_lock<mutex> lock(mMutexBMap);
    mvpReferenceMapPoints = vpMPs;
}

BundledMap::bkfptr BundledMap::GetBKfsPtr(size_t BKfsId, size_t ClientId, bool bIgnoreMutex) //Performance: find a better implementation for this method
{
    if(!bIgnoreMutex)
        unique_lock<mutex> lock(mMutexBMap);

    idpair idp = make_pair(BKfsId,ClientId);
    std::map<idpair,bkfptr>::iterator mit = mmpBundledKeyFrames.find(idp);
    if(mit != mmpBundledKeyFrames.end()) return mit->second;
    else return nullptr;
}

BundledMap::bkfptr BundledMap::GetErasedBKfsPtr(size_t BKfsId, size_t ClientId)
{
    unique_lock<mutex> lock(mMutexBMap);

    idpair idp = make_pair(BKfsId,ClientId);
    std::map<idpair,bkfptr>::iterator mit = mmpErasedBundledKeyFrames.find(idp);
    if(mit != mmpErasedBundledKeyFrames.end()) return mit->second;
    else return nullptr;
}

BundledMap::bkfptr BundledMap::GetRandBKfPtr()
{
    ccptr pCC = *(mspCC.begin());

    if(mnMaxBKFsid < (params::mapping::miNumRecentKFs))
        return nullptr;

    int cnt = 0;
    int MaxIts = 1;
    bkfptr pBKF;

    while(!pBKF && cnt < MaxIts)
    {
        //BKF ID
        size_t min = 0; //this KF is the query KF, it's neighbors are candidates for culling -- so 0 and 1 can be considered
        size_t max = mnMaxBKFsid;
        size_t id = min + (rand() % (size_t)(max - min + 1));

        //Client ID
        size_t cid = MAPRANGE;
        if(msuAssClients.size() > 1)
        {
            min = 0;
            max = msuAssClients.size() - 1;
            size_t temp = min + (rand() % (size_t)(max - min + 1));

            set<size_t>::iterator sit = msuAssClients.begin();
            for(int it = 0;it < msuAssClients.size();++it)
            {
                if(it == temp)
                {
                    cid = *sit;
                    break;
                }
                else
                    ++sit;
            }
        }
        else
        {
            cid = *(msuAssClients.begin());
        }

        pBKF = this->GetBKfsPtr(id,cid);
        ++cnt;
    }

    return pBKF;
}

vector<BundledMap::bkfptr> BundledMap::GetAllBundledKeyFrames()
{
    unique_lock<mutex> lock(mMutexBMap);

    vector<bkfptr> vpBKFs;
    for(std::map<idpair,bkfptr>::iterator mit_set = mmpBundledKeyFrames.begin();mit_set!=mmpBundledKeyFrames.end();++mit_set)
        vpBKFs.push_back(mit_set->second);
    return vpBKFs;
}

vector<BundledMap::mpptr> BundledMap::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexBMap);

    vector<mpptr> vpMPs;
    for(std::map<idpair,mpptr>::iterator mit_set = mmpMapPoints.begin();mit_set!=mmpMapPoints.end();++mit_set)
        vpMPs.push_back(mit_set->second);
    return vpMPs;
}

long unsigned int BundledMap::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexBMap);

    return mmpMapPoints.size();
}


long unsigned int BundledMap::BundledKeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexBMap);
    return mmpBundledKeyFrames.size();
}

long unsigned int BundledMap::GetMaxBKFid()
{
    unique_lock<mutex> lock(mMutexBMap);
    return mnMaxBKFsid;
}

long unsigned int BundledMap::GetMaxMPid()
{
    unique_lock<mutex> lock(mMutexBMap);
    return mnMaxMPid;
}

long unsigned int BundledMap::GetMaxBKFidUnique()
{
    unique_lock<mutex> lock(mMutexBMap);
    return mnMaxBKFsidUnique;
}

long unsigned int BundledMap::GetMaxMPidUnique()
{
    unique_lock<mutex> lock(mMutexBMap);
    return mnMaxMPidUnique;
}

long unsigned int BundledMap::GetLastBKfIdUnique()
{
    unique_lock<mutex> lock(mMutexBMap);
    return mnLastBKfsIdUnique;
}

BundledMap::mpptr BundledMap::GetMpPtr(size_t MpId, size_t ClientId)
{
    unique_lock<mutex> lock(mMutexBMap);

    idpair idp = make_pair(MpId,ClientId);
    std::map<idpair,mpptr>::iterator mit = mmpMapPoints.find(idp);
    if(mit != mmpMapPoints.end()) return mit->second;
    else return nullptr;
}

void BundledMap::AddCCPtr(ccptr pCC)
{
    unique_lock<mutex> lock(this->mMutexCC);
    mspCC.insert(pCC);

    if(pCC->mpCH->GetCommPtr()) //when this is called during init procedure, mpCH->GetCommPtr() is still nullptr
        mspComm.insert(pCC->mpCH->GetCommPtr());
}

set<BundledMap::ccptr> BundledMap::GetCCPtrs()
{
    unique_lock<mutex> lock(this->mMutexCC);
    return mspCC;
}

void BundledMap::StopGBA()
{
    {
        //prevent two handlers from stopping GBA at the same time
        unique_lock<mutex> lockStopGBAInProgress(mMutexStopGBAInProgess);
        if(mbStopGBAInProgress)
            return;
        else
            mbStopGBAInProgress = true;
    }

    cout << "BMap " << mBMapId << ": Stop GBA" << endl;
    if(this->isRunningGBA())
    {
        #ifdef DONOTINTERRUPTMERGE
        if(this->isMergeStepGBA())
        {
            cout << "BMap " << mBMapId << ": GBA stop declined -- MergeGBA" << endl;
            return;
        }
        #endif

        this->mbStopGBA = true;

        while(!this->isFinishedGBA())
            usleep(5000);

        this->mpThreadGBA->join();
        delete this->mpThreadGBA;
    }
    else
    {
        cout << COUTERROR << "called w/o GBA running -- BMap " << mBMapId << endl;
    }

    {
        unique_lock<mutex> lockStopGBAInProgress(mMutexStopGBAInProgess);
        mbStopGBAInProgress = false;
    }
    cout << "BMap " << mBMapId << ": GBA Stopped" << endl;
}

void BundledMap::RequestBA(size_t nClientId)
{
    if(this->isRunningGBA())
    {
        cout << "Denied -- GBA running" << endl;
        return;
    }

    if(this->isNoStartGBA())
    {
        cout << "Denied -- NoStartGBA" << endl;
        return;
    }

    #ifdef FINALBA
    if(!this->isGBAinterrupted())
        cout << COUTERROR << "Agent " << nClientId << " requesting BA, but was not interrupted" << endl;
    #endif

    msnFinishedAgents.insert(nClientId);

    if(msnFinishedAgents.size() == msuAssClients.size())
    {
        bool b0 = false;
        bool b1 = false;
        bool b2 = false;
        bool b3 = false;

        for(set<ccptr>::iterator sit = mspCC.begin();sit!=mspCC.end();++sit)
        {
            ccptr pCC = *sit;

            if(pCC->mClientId > 3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId out of bounds (" << pCC->mClientId << ")" << endl;
            if(!(this->msuAssClients.count(pCC->mClientId))) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId in pCC but not in msuAssClients" << endl;
            switch(pCC->mClientId)
            {
                case(static_cast<size_t>(0)):
                    if(b0) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b0 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                case(static_cast<size_t>(1)):
                    if(b1) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b1 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                case(static_cast<size_t>(2)):
                    if(b2) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b2 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                case(static_cast<size_t>(3)):
                    if(b3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId found twice" << endl;
                    b3 = true;
                    #ifdef LOGGING
//                    pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                    #endif
                    while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                    break;
                default: cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": associated ClientId out of bounds" << endl;
            }

            pCC->mbOptActive = true;
        }

        idpair nLoopBKF = make_pair(mnLastBKfsIdUnique,mBMapId);

        this->setRunningGBA();
        this->setFinishedGBA();
        this->mbStopGBA = false;

        // Launch a new thread to perform Global Bundle Adjustment
        this->mpThreadGBA = new thread(&BundledMap::RunGBA,this,nLoopBKF);
    }
    else
    {
        cout << "msuAssClient: " << endl;
        for(set<size_t>::iterator sit = msuAssClients.begin();sit!=msuAssClients.end();++sit)
            cout << *sit;
        cout << endl;

        cout << "msuFinishedAgents: " << endl;
        for(set<size_t>::iterator sit = msnFinishedAgents.begin();sit!=msnFinishedAgents.end();++sit)
            cout << *sit;
        cout << endl;
    }
}

void BundledMap::RunGBA(idpair nLoopBKF)
{
    cout << "-> Starting Global Bundle Adjustment" << endl;

    Optimizer::BMapFusionGBA(shared_from_this(),this->mBMapId,params::opt::mGBAIterations,&(this->mbStopGBA),nLoopBKF,true);

    #ifdef FINALBA
    if(!this->mbStopGBA)
    #endif
    {
        unique_lock<mutex> lock(this->mMutexGBA);

        this->LockBMapUpdate();

        cout << "-> Global Bundle Adjustment finished" << endl;
        cout << "-> Updating Bmap ..." << endl;

        // Correct keyframes starting at map first keyframe
        list<bkfptr> lpBKFtoCheck(this->mvpBundledKeyFramesOrigins.begin(),this->mvpBundledKeyFramesOrigins.end());

        cout << "--> Updating BKFs ..." << endl;

        while(!lpBKFtoCheck.empty())
        {
            bkfptr pBKF = lpBKFtoCheck.front();
            const set<bkfptr> sChilds = pBKF->GetChilds();
            cv::Mat Twc = pBKF->GetPoseInverse();
            for(set<bkfptr>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
            {
                bkfptr pChild = *sit;
                if(pChild->mBAGlobalForBKFs!=nLoopBKF)
                {
                    cv::Mat Tchildc = pChild->GetPose()*Twc;
                    pChild->mTcwGBA = Tchildc*pBKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                    pChild->mBAGlobalForBKFs=nLoopBKF;

                }
                lpBKFtoCheck.push_back(pChild);
            }

           
            pBKF->mTcwBefGBA = pBKF->GetPose();
           
            pBKF->SetPose(pBKF->mTcwGBA,true);
            pBKF->mbLoopCorrected = true;
            lpBKFtoCheck.pop_front();
        }

        cout << "--> Updating MPs ..." << endl;

        // Correct MapPoints
        const vector<mpptr> vpMPs = this->GetAllMapPoints();

        for(size_t i=0; i<vpMPs.size(); i++)
        {
            mpptr pMP = vpMPs[i];

            if(pMP->isBad())
                continue;

            if(pMP->mBAGlobalForBKFs==nLoopBKF)
            {
                // If optimized by Global BA, just update
                pMP->SetWorldPos(pMP->mPosGBA,true);
                pMP->mbLoopCorrected = true;
            }
            else
            {
                // Update according to the correction of its reference keyframe
                bkfptr pRefBKF = pMP->GetReferenceBundledKeyFrame();

                if(!pRefBKF)
                {
                    cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": pRefKf is nullptr" << endl;
                    continue;
                }

                if(pRefBKF->mBAGlobalForBKFs!=nLoopBKF)
                    continue;

                // Map to non-corrected camera
                cv::Mat Rcw = pRefBKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                cv::Mat tcw = pRefBKF->mTcwBefGBA.rowRange(0,3).col(3);
                cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                // Backproject using corrected camera
                cv::Mat Twc = pRefBKF->GetPoseInverse();
                cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                cv::Mat twc = Twc.rowRange(0,3).col(3);

                pMP->SetWorldPos(Rwc*Xc+twc,true);
                pMP->mbLoopCorrected = true;
            }
        }

        cout << "-> BMap updated!" << endl;

        #ifdef FINALBA
        this->unsetGBAinterrupted();
        #endif

        this->UnLockBMapUpdate();
    }
    #ifdef FINALBA
    else
    {
        cout << COUTNOTICE << "GBA interrupted" << endl;
        this->setGBAinterrupted();
    }
    #endif
    //todo
    // if(params::stats::mbWriteKFsToFile)
    // {
    //     for(int it=0;it<4;++it)
    //     {
    //         std::stringstream ss;
    //         ss << params::stats::msOutputDir << "KF_GBA_" << it << ".csv";
    //         this->WriteStateToCsv(ss.str(),it);
    //     }
    // }

    this->setFinishedGBA();
    this->unsetRunningGBA();

    for(set<ccptr>::iterator sit = mspCC.begin();sit!=mspCC.end();++sit)
    {
        ccptr pCC = *sit;
        pCC->UnLockMapping();

        pCC->mbOptActive = false;
    }

    cout << "-> Leave Thread" << endl;
}


void BundledMap::ClearBadMPs()
{
    unique_lock<mutex> lock(mMutexBMap);
    unique_lock<mutex> lock2(mMutexErased);

    for(set<mpptr>::iterator sit = mspMPsToErase.begin();sit != mspMPsToErase.end();)
    {
        mpptr pMPi = *sit;

        if(pMPi)
        {
            map<idpair,mpptr>::iterator mit2 = mmpMapPoints.find(pMPi->mId);
            if(mit2 != mmpMapPoints.end())
                mmpMapPoints.erase(mit2);
        }

        sit = mspMPsToErase.erase(sit);
    }
}

BundledMap::ccptr BundledMap::GetCCPtr(size_t nClientId)
{
    for(set<ccptr>::iterator sit = mspCC.begin();sit != mspCC.end(); ++sit)
    {
        ccptr pCC = *sit;
        if(pCC->mClientId == nClientId)
            return pCC;
    }

    cout << COUTERROR << "no ccptr found for query-ID" << endl;
    return nullptr;
}

void BundledMap::MapTrimming(bkfptr pBKFcur)
{
    unique_lock<mutex> lock(mMutexBMap);
    unique_lock<mutex> lock2(mMutexErased);

    int nLocalBKFs = params::mapping::miLocalMapSize;
    int BKfsLimit = params::mapping::miLocalMapSize + params::mapping::miLocalMapBuffer;

    if(mmpBundledKeyFrames.size() <= nLocalBKFs)
        return;

    set<bkfptr> sBKfsVicinity;
    set<mpptr> sMpVicinity;
    sBKfsVicinity.insert(pBKFcur);

    //for KF Limit
    priority_queue<int> pqNativeBKFs;
    list<bkfptr> lForeignBKFs;

    pBKFcur->UpdateConnections(true);

    this->FindLocalBKFsByTime(pBKFcur,sBKfsVicinity,pqNativeBKFs,lForeignBKFs,nLocalBKFs);

    //add MPs included by the BKFs
    for(set<bkfptr>::iterator sit = sBKfsVicinity.begin();sit!=sBKfsVicinity.end();++sit)
    {
        bkfptr pBKFi = *sit;
        vector<mpptr> vMPs = pBKFi->GetMapPointMatches();

        sMpVicinity.insert(vMPs.begin(),vMPs.end());
    }

    //find & erase BKFs

    for(map<idpair,bkfptr>::iterator mit = mmpBundledKeyFrames.begin();mit!=mmpBundledKeyFrames.end();)
    {
        bkfptr pBKFi = mit->second;
        bool bErase = false;

        if(!sBKfsVicinity.count(pBKFi))
        {
            if(pBKFi->CanBeForgotten() || pBKFi->mId.second != this->mBMapId || pBKFi->isBad())
                bErase = true;
        }

        if(bErase)
        {
            if(!pBKFi->isBad())
                pBKFi->SetBadFlag(true);

            if(!pBKFi->isBad())
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " SetBadFlagBKF() called, but BKF not set to bad" << endl;
                throw infrastructure_ex();
            }

            if(pBKFi->mId.second == mBMapId) mmpErasedBundledKeyFrames[pBKFi->mId] = pBKFi;

            mit = mmpBundledKeyFrames.erase(mit);
        }
        else
            ++mit;
    }

    //find & erase MPs
    for(map<idpair,mpptr>::iterator mit = mmpMapPoints.begin();mit!=mmpMapPoints.end();)
    {
        mpptr pMPi = mit->second;
        bool bErase = false;

        if(!sMpVicinity.count(pMPi))
        {
            if(pMPi->CanBeForgotten() || pMPi->mId.second != this->mBMapId || pMPi->isBad())
                bErase = true;
        }

        if(bErase || pMPi->isBad())
        {
            if(!pMPi->isBad())
                pMPi->SetBadFlagBKFs(true);

            if(pMPi->mId.second == mBMapId) mmpErasedMapPoints[pMPi->mId] = pMPi;

            mit = mmpMapPoints.erase(mit);

            if(mspMPsToErase.count(pMPi))
                mspMPsToErase.erase(pMPi);
        }
        else
        {
            ++mit;
        }
    }

    for(set<mpptr>::iterator sit = mspMPsToErase.begin();sit != mspMPsToErase.end();)
    {
        mpptr pMPi = *sit;

        if(pMPi)
        {
            std::map<idpair,mpptr>::iterator mit2 = mmpMapPoints.find(pMPi->mId);
            if(mit2 != mmpMapPoints.end())
                mmpMapPoints.erase(mit2);
        }

        sit = mspMPsToErase.erase(sit);
    }

    if(!mspMPsToErase.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " mspMPsToErase !empty()" << endl;
        throw infrastructure_ex();
    }

    //------------------------
    //--- Enforce BKF Limit ---
    //------------------------

    if(mmpBundledKeyFrames.size() > BKfsLimit)
    {
        cout << "+++++ Enforcing BKF upper limit +++++" << BKfsLimit << endl;

        set<bkfptr,bkftimecmpsmaller>spBKFsmintime;
        for(std::map<idpair,bkfptr>::iterator mit_set = mmpBundledKeyFrames.begin();mit_set!=mmpBundledKeyFrames.end();++mit_set)
            spBKFsmintime.insert(mit_set->second);

        while(mmpBundledKeyFrames.size() > BKfsLimit)
        {
            if(!lForeignBKFs.empty())
            {
                bkfptr pBKFi = lForeignBKFs.front();

                //can always be forgotten, comes from server
                if(pBKFi->mId.second == mBMapId)
                    mmpErasedBundledKeyFrames[pBKFi->mId] = pBKFi;

                std::map<idpair,bkfptr>::iterator mit = mmpBundledKeyFrames.find(pBKFi->mId);
                if(mit != mmpBundledKeyFrames.end()) mmpBundledKeyFrames.erase(mit);

                lForeignBKFs.pop_front();

                spBKFsmintime.erase(pBKFi);
            }
            else if(!mmpBundledKeyFrames.empty())
            {
                bkfptr pBKFi = *(spBKFsmintime.begin());

                if(pBKFi->mId.first == 0)
                {
                    //do nothing -- never delete 0
                    spBKFsmintime.erase(pBKFi);
                }
                else
                {
                    pBKFi->SetBadFlag(true);

                    if(pBKFi->mId.second == mBMapId) mmpErasedBundledKeyFrames[pBKFi->mId] = pBKFi;

                    std::map<idpair,bkfptr>::iterator mit = mmpBundledKeyFrames.find(pBKFi->mId);
                    if(mit != mmpBundledKeyFrames.end())
                        mmpBundledKeyFrames.erase(mit);
                    else
                        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " MP ins mspKeyFrames, but not in mspKeyFrames" << endl;

                    spBKFsmintime.erase(pBKFi);
                }
            }
            else
                cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ": only current BKF left, and the ones that cannot be erased" << endl;
        }

        //find MPs included by the BKFs
        set<mpptr> spMPsToKeep;
        for(map<idpair,bkfptr>::iterator mit = mmpBundledKeyFrames.begin();mit!=mmpBundledKeyFrames.end();++mit)
        {
            bkfptr pBKFi = mit->second;

            vector<mpptr> vMPs = pBKFi->GetMapPointMatches();

            for(vector<mpptr>::iterator vit = vMPs.begin();vit!=vMPs.end();++vit)
            {
                mpptr pMPi = *vit;
                if(!pMPi || pMPi->isBad()) continue;

                spMPsToKeep.insert(pMPi);
            }
        }

        //delete other MPs
        for(map<idpair,mpptr>::iterator mit = mmpMapPoints.begin();mit!=mmpMapPoints.end();)
        {
            mpptr pMPi = mit->second;

            if(!spMPsToKeep.count(pMPi))
            {
                pMPi->SetBadFlagBKFs(true);

                if(pMPi->mId.second == mBMapId) mmpErasedMapPoints[pMPi->mId] = pMPi;
                mit = mmpMapPoints.erase(mit);
            }
            else
                ++mit;
        }

        std::cout << "BKFs after erasing: " << mmpBundledKeyFrames.size() << std::endl;
    }
}

bool BundledMap::bkftimecmp::operator ()(const bkfptr pA, const bkfptr pB) const
{
    return pA->mdInsertStamp > pB->mdInsertStamp;
}

bool BundledMap::bkftimecmpsmaller::operator ()(const bkfptr pA, const bkfptr pB) const
{
    return pA->mdInsertStamp < pB->mdInsertStamp;
}


void BundledMap::FindLocalBKFsByTime(bkfptr pBKFcur,set<bkfptr>& sBKfsVicinity,priority_queue<int>& pqNativeBKFs,list<bkfptr>& lForeignBKFs, int nLocalBKFs)
{
    set<bkfptr,bkftimecmp> spBKFssort;

    for(std::map<idpair,bkfptr>::iterator mit_set = mmpBundledKeyFrames.begin();mit_set!=mmpBundledKeyFrames.end();++mit_set)
        spBKFssort.insert(mit_set->second);

    //keep n newest BKFs
    {
        int nMax = spBKFssort.size(); //cannot use 'spKFsort.size()' in loop header because size continuously reduces in loop itself
        for(int it=0;it<nMax;++it)
        {
            if(spBKFssort.empty())
                break;

            bkfptr pBKFi = *(spBKFssort.begin());
            spBKFssort.erase(pBKFi);

            if(!pBKFi || pBKFi->isBad())
                continue;

            sBKfsVicinity.insert(pBKFi);

            if(pBKFi->mId.second == mBMapId)
                pqNativeBKFs.push(pBKFi->mId.first);
            else
                lForeignBKFs.push_back(pBKFi);

            if(sBKfsVicinity.size() >= nLocalBKFs)
                break;
        }
    }
}

BundledMap::bkfptr BundledMap::GetPredecessor(bkfptr pBKFs)
{
    bkfptr pPred;
    size_t bkfid = pBKFs->mId.first;
    while(!pPred)
    {
        bkfid--;

        if(bkfid == -1)
        {
            cout << "\033[1;31m!!!!! FATAL !!!!!\033[0m " << __func__ << __LINE__ << " cannot find predecessor" << endl;
            cout << "BKF ID: " << pBKFs->mId.first << "|" << pBKFs->mId.second << endl;
            cout << "In map: " << this->mnMaxBKFsid << endl;
            cout << "Workaround: take first BKF (id 0)" << endl;
            pPred = this->GetBKfsPtr(0,mBMapId);
            cout << "get BKF: 0|" << mBMapId <<" -- nullptr? " << (int)!pPred << endl;
        }
        else
        {
            pPred = this->GetBKfsPtr(bkfid,pBKFs->mId.second);
        }
    }

    return pPred;
}

}
