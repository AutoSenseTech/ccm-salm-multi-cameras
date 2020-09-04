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

}
