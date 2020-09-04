#include <cslam/BundledKeyFrames.h>
#define NEWSENDING

namespace cslam {
size_t BundledKeyFrames::nNextId=0;


//for multi camera
BundledKeyFrames::BundledKeyFrames(Frame &F, const vector<kfptr> &vKeyFrames, bmapptr pBMap, bdbptr pBKFDB, commptr pComm, eSystemState SysState, size_t UniqueId):
        cameraNum(F.cameraNum),mvpKeyFrames(vKeyFrames), mvTcamji(F.mvTcamji), vKeyPointsIndexMapPlus(F.vKeyPointsIndexMapPlus), 
        mFrameId(F.mId),mUniqueId(UniqueId),mTimeStamp(F.mTimeStamp),mVisId(-1),
        mTrackReferenceForFrame(defpair), mFuseTargetForBKFs(defpair),
        mnLoopWords(0), mRelocQuery(defpair), mnRelocWords(0),
        vfx(F.vfx), vfy(F.vfy), vcx(F.vcx), vcy(F.vcy), vinvfx(F.vinvfx), vinvfy(F.vinvfy),
        N(F.N_L_R), mDescriptors(F.mJointDescriptors.clone()), mvBDepth(F.mvBDepth),
        mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
        mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
        mvInvLevelSigma2(F.mvInvLevelSigma2), mvpMapPoints(F.mvpMapPointsBKFs),
        mbFirstConnection(true), mpParent(nullptr), mbNotErase(false),
        mbToBeErased(false), mbBad(false),
        mpORBvocabulary(F.mpORBvocabulary), mpBMap(pBMap), mpBundledKeyFramesDB(pBKFDB),
        mbIsEmpty(false),mbPoseLock(false),mbPoseChanged(false),mbInOutBuffer(false), mbSentOnce(false),
        mLoopQuery(defpair),mMatchQuery(defpair),
        mBALocalForBKFs(defpair),mBAFixedForBKFs(defpair),mBAGlobalForBKFs(defpair),
        mSysState(SysState),mbOmitSending(false),
        mbAck(false),mbFromServer(false),mbUpdatedByServer(false),mCorrected_MM(defpair),mbSendFull(true)     
{
    mId=make_pair(nNextId++,F.mId.second);

    mspComm.insert(pComm);

    mvbMapPointsLock.resize(N,false);

    if(mUniqueId == 0 && !mId.first == 0) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::KeyFrame(..): mUniqueId not set" << endl;

    mdInsertStamp = ros::Time::now().toNSec();
}

void BundledKeyFrames::AddMapPoint(mpptr pMP, const size_t &index, bool bLock)
{
    unique_lock<mutex> lock(mMutexFeatures);

    if(mvbMapPointsLock[index] && mSysState == eSystemState::CLIENT)
    {
        return;
    }

    mpptr pMPold = mvpMapPoints[index];
    if(pMPold)
    {
        pMPold->EraseBKFsObservation(shared_from_this(), cameraNum);
     
    }
        

    mvpMapPoints[index]=pMP;

    if(bLock)
    {
        mvbMapPointsLock[index] = true;
    }
}

void BundledKeyFrames::EraseMapPointMatch(const size_t &index, bool bLock)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(this->IsEmpty()) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::EraseMapPointMatch(): trying to erase MP from empty KF" << endl;

    if(mvbMapPointsLock[index] && mSysState == eSystemState::CLIENT)
    {
        return;
    }

    mvpMapPoints[index]=nullptr;

    if(bLock)
    {
        mvbMapPointsLock[index] = true;
    }
}

void BundledKeyFrames::EraseMapPointMatch(mpptr pMP, bool bLock)
{    
    int index = pMP->GetIndexInBundledKeyFrames(this->shared_from_this());
    if(index>=0)
    {
        if(mvbMapPointsLock[index] && mSysState == eSystemState::CLIENT)
        {
            return;
        }

        mvpMapPoints[index]=nullptr;

        if(bLock)
        {
            mvbMapPointsLock[index] = true;
        }
    }
    else
    {
        vector<mpptr>::iterator vit = std::find(mvpMapPoints.begin(),mvpMapPoints.end(),pMP);

        if(vit == mvpMapPoints.end())
        {
            //MP not in KF -- nothing to erase
        }
        else
        {
            //MP thinks it is not in KF, but KF has it associated to feat
            int id = vit - mvpMapPoints.begin();
            mvpMapPoints[id] = nullptr;
        }
    }
}

int BundledKeyFrames::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        mpptr pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

void BundledKeyFrames::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void BundledKeyFrames::AddConnection(bkfptr pBKFs, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedBundledKeyFramesWeights.count(pBKFs))
            mConnectedBundledKeyFramesWeights[pBKFs]=weight;
        else if(mConnectedBundledKeyFramesWeights[pBKFs]!=weight)
            mConnectedBundledKeyFramesWeights[pBKFs]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void BundledKeyFrames::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,bkfptr> > vPairs;
    vPairs.reserve(mConnectedBundledKeyFramesWeights.size());
    for(map<bkfptr,int>::iterator mit=mConnectedBundledKeyFramesWeights.begin(), mend=mConnectedBundledKeyFramesWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<bkfptr> lBKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lBKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedBundledKeyFrames = vector<bkfptr>(lBKFs.begin(),lBKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
}




void BundledKeyFrames::UpdateConnections(bool bIgnoreMutex)
{
    bool bSetBad = false;

    map<bkfptr,int> BKFscounter;

    vector<mpptr> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<mpptr>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        mpptr pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<bkfptr,size_t> observations = pMP->GetBKFsObservations();

        for(map<bkfptr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mId == this->mId)
                continue;
            BKFscounter[mit->first]++;
        }
    }

    if(BKFscounter.empty())
    {
        return;
    }

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    bkfptr pBKFsmax=nullptr;
    int th = 15; // FIXME: 建立共识

    vector<pair<int,bkfptr> > vPairs;
    vPairs.reserve(BKFscounter.size());
    for(map<bkfptr,int>::iterator mit=BKFscounter.begin(), mend=BKFscounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pBKFsmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this->shared_from_this(),mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pBKFsmax));
        pBKFsmax->AddConnection(this->shared_from_this(),nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<bkfptr> lBKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lBKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        mConnectedBundledKeyFramesWeights = BKFscounter;
        mvpOrderedConnectedBundledKeyFrames = vector<bkfptr>(lBKFs.begin(),lBKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mId.first!=0)
        {
            if(mSysState == eSystemState::CLIENT)
            {                
                mpParent = mvpOrderedConnectedBundledKeyFrames.front();
                if(mspChildrens.count(mpParent) || mpParent->mbFromServer);
                {
                    //THIS is alread parent of the desginated parent
                    //Furthermore, do not use KFs sent from Server as parents
                    vector<bkfptr>::iterator vit = mvpOrderedConnectedBundledKeyFrames.begin();
                    while(mspChildrens.count(mpParent) || mpParent->mbFromServer)
                    {
                        ++vit;
                        if(vit == mvpOrderedConnectedBundledKeyFrames.end())
                        {
                            if(this->mId.second == mpBMap->mBMapId)
                            {
                                bSetBad = true;
                                break;
                            }
                            else
                            {
                                mpParent = nullptr;
                                break;
                            }
                        }
                        mpParent = *vit;
                    }
                }
            }
            else if(mSysState == eSystemState::SERVER)
            {
                //Enforce tree structure on server
                vector<bkfptr>::iterator vit = mvpOrderedConnectedBundledKeyFrames.begin();
                bkfptr pPC = *vit;
                while(!(pPC->mId.first < this->mId.first))
                {
                    ++vit;
                    if(vit == mvpOrderedConnectedBundledKeyFrames.end())
                    {
                        //strategy: use nearest predecessor
                        {
                            for(int itid=1;itid<10;itid++)
                            {
                                pPC = mpBMap->GetBKfsPtr(mId.first-itid,mId.second);
                                if(pPC)
                                    break;
                            }

                            if(!pPC)
                            {
                                std::cout << "No predecessor" << std::endl;
                                throw estd::infrastructure_ex();
                            }

                            break;
                        }
                    }
                    pPC = *vit;
                }
                mpParent = pPC;
            }
            if(!bSetBad)
            {
                if(mpParent)
                {
                    mpParent->AddChild(this->shared_from_this());
                    mbFirstConnection = false;
                }
                else if(!(mSysState == eSystemState::CLIENT && this->mId.second != mpBMap->mBMapId))
                {
                    cout << "\033[1;31m!!!!! FATAL !!!!!\033[0m " << __func__ << ":" << __LINE__ << ": cannot find parent" << endl;
                    throw infrastructure_ex();
                }
                else
                {
                    //On the client and cannot process KF from other client -- ignore data
                    {
                        for(map<bkfptr,int>::iterator mit = mConnectedBundledKeyFramesWeights.begin(), mend=mConnectedBundledKeyFramesWeights.end(); mit!=mend; mit++)
                            mit->first->EraseConnection(this->shared_from_this());
                    }

                    for(size_t i=0; i<mvpMapPoints.size(); i++)
                    {
                        if(mvpMapPoints[i])
                        {
                            mpptr pMPx = mvpMapPoints[i];
                            pMPx->EraseBKFsObservation(this->shared_from_this(),cameraNum,false,true);
                            
                        }
                    }


                    {
                        unique_lock<mutex> lock1(mMutexFeatures);

                        mConnectedBundledKeyFramesWeights.clear();
                        mvpOrderedConnectedBundledKeyFrames.clear();

                        if(!mspChildrens.empty())
                            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " mspChildrens assumed to be empty at this point" << endl;
                        mbBad = true;
                    }

                    mpBMap->EraseBundledKeyFrames(this->shared_from_this());
                    mpBundledKeyFramesDB->erase(this->shared_from_this());
                }
            }
        }
    }

    if(!bSetBad && mpParent && mpParent->mId == this->mId)
    {
        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " child->mId == this->mId" << endl;
        cout << "This: " << mId.first << "|" << mId.second << " -- Parent: " << mpParent->mId.first << "|" << mpParent->mId.second << endl;
        throw infrastructure_ex();
    }

    if(bSetBad)
    {
        this->SetBadFlag(false,true);
    }

    #ifdef DEBUGGING2
    {
        bkfptr pBKFp = this->GetParent();
        if(pBKFp)
        {
            if(pBKFp->mId == this->mId)
            {
                std::cout << COUTERROR << "BKF " << this->mId.first << "|" << this->mId.second << " : is its own parent" << std::endl;
            }
        }
        else
        {
            if(!mId.first==0)
                std::cout << COUTERROR << "BKF " << this->mId.first << "|" << this->mId.second << " : no parent" << std::endl;
        }
    }
    #endif
}

void BundledKeyFrames::SetBadFlag(bool bSuppressMapAction, bool bNoParent)
{

    #ifdef LOGGING
    ccptr pCC;
    if(this->mSysState == SERVER)
    {
        pCC = mpBMap->GetCCPtr(this->mId.second);
        pCC->mpLogger->SetKF(__LINE__,this->mId.second);
    }
    #endif

    {
        if(mbBad)
        {
            #ifdef LOGGING
            if(this->mSysState == SERVER)
                pCC->mpLogger->SetKF(__LINE__,this->mId.second);
            #endif
            return;
        }
    }

    {
        unique_lock<mutex> lock(mMutexConnections);

        if(mId.first==0)
        {
            #ifdef LOGGING
            if(this->mSysState == SERVER)
                pCC->mpLogger->SetKF(__LINE__,this->mId.second);
            #endif
            return;
        }
        else if(mbNotErase)
        {
            if(mSysState == eSystemState::CLIENT)
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " mbNotErase not supposed to be  TRUE at any time on client" << endl;
                throw infrastructure_ex();
            }

            mbToBeErased = true;
            #ifdef LOGGING
            if(this->mSysState == SERVER)
                pCC->mpLogger->SetKF(__LINE__,this->mId.second);
            #endif
            return;
        }

        for(map<bkfptr,int>::iterator mit = mConnectedBundledKeyFramesWeights.begin(), mend=mConnectedBundledKeyFramesWeights.end(); mit!=mend; mit++)
            mit->first->EraseConnection(this->shared_from_this());
    }

    for(size_t i=0; i<mvpMapPoints.size(); i++)
    {
        if(mvpMapPoints[i])
        {
                mpptr pMPx = mvpMapPoints[i];

                pMPx->EraseBKFsObservation(this->shared_from_this(),cameraNum, false,true);
        }
    }

    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedBundledKeyFramesWeights.clear();
        mvpOrderedConnectedBundledKeyFrames.clear();

        // Update Spanning Tree
        set<bkfptr> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            bkfptr pC;
            bkfptr pP;

            for(set<bkfptr>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                bkfptr pBKFs = *sit;
                if(pBKFs->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<bkfptr> vpConnected = pBKFs->GetVectorCovisibleBundledKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<bkfptr>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(pBKFs->hasChild(*spcit))
                            continue;

                        if(mSysState == eSystemState::SERVER)
                        {
                            //Enforce tree structure on server
                            if(!((*spcit)->mId < pBKFs->mId))
                                continue;
                        }

                        if((*spcit)->mbFromServer)
                            continue; // do not use KFs from Server as parents

                        if(vpConnected[i]->mId == (*spcit)->mId)
                        {
                            int w = pBKFs->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pBKFs;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }

                #ifdef DEBUGGING2
                {
                    bkfptr pBKFsp = pBKFs->GetParent();
                    if(pBKFsp)
                    {
                        if(pBKFsp->mId == pBKFs->mId)
                        {
                            std::cout << COUTERROR << "BKF " << pBKFs->mId.first << "|" << pBKFs->mId.second << " : is its own parent" << std::endl;
                        }
                    }
                    else
                    {
                        if(!pBKFs->mId.first==0)
                            std::cout << COUTERROR << "BKF " << pBKFs->mId.first << "|" << pBKFs->mId.second << " : no parent" << std::endl;
                    }
                }
                #endif
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
        {
            while(!mspChildrens.empty()) //cannot use "for(set<kfptr>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)" here --> ChangeParent will erase a child, this results in a memory leak
            {

                bkfptr pCi = *(mspChildrens.begin());

                pCi->ChangeParent(mpParent);

                #ifdef DEBUGGING2
                {
                    bkfptr pBKFsp = pCi->GetParent();
                    if(pBKFsp)
                    {
                        if(pBKFsp->mId == pCi->mId)
                        {
                            std::cout << COUTERROR << "BKF " << pCi->mId.first << "|" << pCi->mId.second << " : is its own parent" << std::endl;
                        }
                    }
                    else
                    {
                        if(!pCi->mId.first==0)
                            std::cout << COUTERROR << "BKF " << pCi->mId.first << "|" << pCi->mId.second << " : no parent" << std::endl;
                    }
                }
                #endif
            }
        }

        if(!bNoParent)
        {
            //When BKF comes from Server, and can not acquire MPs, it also has no parent, since there are no connected KFs
            if(mpParent)
            {
                mpParent->EraseChild(this->shared_from_this());
                mTcp = Tcw*mpParent->mvpKeyFrames[0]->GetPoseInverse().clone();
            }
            else
            {
                Tcw.copyTo(mTcp);
                mpParent = mpBMap->GetBKfsPtr(0,mId.second);
            }
        }
        mbBad = true;
    }

    if(!bSuppressMapAction)
        mpBMap->EraseBundledKeyFrames(this->shared_from_this());

    mpBundledKeyFramesDB->erase(this->shared_from_this());

    #ifdef LOGGING
    if(this->mSysState == SERVER)
        pCC->mpLogger->SetKF(__LINE__,this->mId.second);
    #endif
}

void BundledKeyFrames::EraseConnection(bkfptr pBKFs)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedBundledKeyFramesWeights.count(pBKFs))
        {
            mConnectedBundledKeyFramesWeights.erase(pBKFs);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

void BundledKeyFrames::AddChild(bkfptr pBKFs)
{
    unique_lock<mutex> lockCon(mMutexConnections);

    mspChildrens.insert(pBKFs);
}


vector<BundledKeyFrames::bkfptr> BundledKeyFrames::GetBestCovisibilityBundledKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedBundledKeyFrames.size()<N)
        return mvpOrderedConnectedBundledKeyFrames;
    else
        return vector<bkfptr>(mvpOrderedConnectedBundledKeyFrames.begin(),mvpOrderedConnectedBundledKeyFrames.begin()+N);

}


set<BundledKeyFrames::bkfptr> BundledKeyFrames::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

BundledKeyFrames::bkfptr BundledKeyFrames::GetParent(bool bIgnorePoseMutex)
{
    if(!bIgnorePoseMutex)
        unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

vector<BundledKeyFrames::mpptr> BundledKeyFrames::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

BundledKeyFrames::mpptr BundledKeyFrames::GetMapPoint(const size_t &index)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[index];
}


vector<BundledKeyFrames::bkfptr> BundledKeyFrames::GetVectorCovisibleBundledKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedBundledKeyFrames;
}

bool BundledKeyFrames::hasChild(bkfptr pBKFs)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pBKFs);
}

int BundledKeyFrames::GetWeight(bkfptr pBKFs)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedBundledKeyFramesWeights.count(pBKFs))
        return mConnectedBundledKeyFramesWeights[pBKFs];
    else
        return 0;
}

void BundledKeyFrames::ChangeParent(bkfptr pBKFs)
{
    unique_lock<mutex> lockCon(mMutexConnections);

    if(mpParent)
    {
        mpParent->EraseChild(shared_from_this(),true);
    }

    mpParent = pBKFs;

    pBKFs->AddChild(this->shared_from_this());
}

void BundledKeyFrames::EraseChild(bkfptr pBKFs, bool bIgnoreMutex)
{
    if(!bIgnoreMutex)
        unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pBKFs);
}

void BundledKeyFrames::ReplaceMapPointMatch(const size_t &index, mpptr pMP, bool bLock, bool bOverrideLock)
{
    unique_lock<mutex> lock(mMutexFeatures); //was not here in ORB implementation

    if(mvbMapPointsLock[index] && mSysState == eSystemState::CLIENT)
        return;

    if(mvpMapPoints[index] && mvpMapPoints[index]->mbDoNotReplace) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::ReplaceMapPointMatch(...): mbDoNotReplace true" << endl;

    mvpMapPoints[index]=pMP;

    if(bLock)
    {
        mvbMapPointsLock[index] = true;
    }
}

bool BundledKeyFrames::CanBeForgotten()
{
    if(mId.first == 0)
        return false;
    unique_lock<mutex> lock(mMutexOut);

    if(mbSentOnce && mbAck && !mbInOutBuffer)
        return true;
    else
        return false;
}
}