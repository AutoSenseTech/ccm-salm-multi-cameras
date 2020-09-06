#include <cslam/BundledKeyFrames.h>
#define NEWSENDING

namespace cslam {
size_t BundledKeyFrames::nNextId=0;


// //for multi camera
// BundledKeyFrames::BundledKeyFrames(Frame &F, const vector<kfptr> &vKeyFrames, bmapptr pBMap, bdbptr pBKFDB, commptr pComm, eSystemState SysState, size_t UniqueId):
//         cameraNum(F.cameraNum),mvpKeyFrames(vKeyFrames), mvTcamji(F.mvTcamji), vKeyPointsIndexMapPlus(F.vKeyPointsIndexMapPlus), 
//         mFrameId(F.mId),mUniqueId(UniqueId),mTimeStamp(F.mTimeStamp),mVisId(-1),
//         mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
//         mvGridElementWidthInv(F.mvGridElementWidthInv), mvGridElementHeightInv(F.mvGridElementHeightInv),
//         mTrackReferenceForFrame(defpair), mFuseTargetForBKFs(defpair),
//         mnLoopWords(0), mRelocQuery(defpair), mnRelocWords(0),
//         vfx(F.vfx), vfy(F.vfy), vcx(F.vcx), vcy(F.vcy), vinvfx(F.vinvfx), vinvfy(F.vinvfy),
//         mvpkeyPointsNum(F.mvpkeyPointsNum),mvKeysMultipleUn(F.mvKeysMultipleUn), mvpDescriptors(F.mvpDescriptors),
//         N(F.N_L_R), mDescriptors(F.mJointDescriptors.clone()), mvBDepth(F.mvBDepth),
//         mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
//         mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
//         mvInvLevelSigma2(F.mvInvLevelSigma2), mvpMapPoints(F.mvpMapPointsBKFs),
//         mbFirstConnection(true), mpParent(nullptr), mbNotErase(false),
//         mbToBeErased(false), mbBad(false),
//         mpORBvocabulary(F.mpORBvocabulary), mpBMap(pBMap), mpBundledKeyFramesDB(pBKFDB),
//         mbIsEmpty(false),mbPoseLock(false),mbPoseChanged(false),mbInOutBuffer(false), mbSentOnce(false),
//         mLoopQuery(defpair),mMatchQuery(defpair),
//         mBALocalForBKFs(defpair),mBAFixedForBKFs(defpair),mBAGlobalForBKFs(defpair),
//         mSysState(SysState),mbOmitSending(false),mbLoopCorrected(false),
//         mbAck(false),mbFromServer(false),mbUpdatedByServer(false),mCorrected_MM(defpair),mbSendFull(true)     
// {
//     mId=make_pair(nNextId++,F.mId.second);

//     mspComm.insert(pComm);

//     mGrid0.resize(mnGridCols);
//     for(int i=0; i<mnGridCols;i++)
//     {
//         mGrid0[i].resize(mnGridRows);
//         for(int j=0; j<mnGridRows; j++)
//             mGrid0[i][j] = F.mGrid0[i][j];
//     }

//     mGrid1.resize(mnGridCols);
//     for(int i=0; i<mnGridCols;i++)
//     {
//         mGrid1[i].resize(mnGridRows);
//         for(int j=0; j<mnGridRows; j++)
//             mGrid1[i][j] = F.mGrid1[i][j];
//     }

//     // mGrid2.resize(mnGridCols);
//     // for(int i=0; i<mnGridCols;i++)
//     // {
//     //     mGrid2[i].resize(mnGridRows);
//     //     for(int j=0; j<mnGridRows; j++)
//     //         mGrid2[i][j] = F.mGrid2[i][j];
//     // }

//     // mGrid3.resize(mnGridCols);
//     // for(int i=0; i<mnGridCols;i++)
//     // {
//     //     mGrid3[i].resize(mnGridRows);
//     //     for(int j=0; j<mnGridRows; j++)
//     //         mGrid3[i][j] = F.mGrid3[i][j];
//     // }

//     SetPose(F.mTcw,false);

//     mvbMapPointsLock.resize(N,false);

//     if(mUniqueId == 0 && !mId.first == 0) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::KeyFrame(..): mUniqueId not set" << endl;

//     mdInsertStamp = ros::Time::now().toNSec();
// }

//for multi camera
BundledKeyFrames::BundledKeyFrames(Frame &F, bmapptr pBMap, bdbptr pBKFDB, commptr pComm, eSystemState SysState, size_t UniqueId):
        cameraNum(F.cameraNum), mvTcamji(F.mvTcamji), vKeyPointsIndexMapPlus(F.vKeyPointsIndexMapPlus), 
        mFrameId(F.mId),mUniqueId(UniqueId),mTimeStamp(F.mTimeStamp),mVisId(-1),
        mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
        mvGridElementWidthInv(F.mvGridElementWidthInv), mvGridElementHeightInv(F.mvGridElementHeightInv),
        mTrackReferenceForFrame(defpair), mFuseTargetForBKFs(defpair),
        mnLoopWords(0), mRelocQuery(defpair), mnRelocWords(0),
        vfx(F.vfx), vfy(F.vfy), vcx(F.vcx), vcy(F.vcy), vinvfx(F.vinvfx), vinvfy(F.vinvfy),
        mvpkeyPointsNum(F.mvpkeyPointsNum),mvKeysMultipleUn(F.mvKeysMultipleUn), mvpDescriptors(F.mvpDescriptors),
        N(F.N_L_R), mDescriptors(F.mJointDescriptors.clone()), mvBDepth(F.mvBDepth),
        mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
        mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
        mvInvLevelSigma2(F.mvInvLevelSigma2), mvMinX(F.mvMinX), mvMinY(F.mvMinY), mvMaxX(F.mvMaxX),
         mvMaxY(F.mvMaxY), mvpK(F.mvpK),
        mvpMapPoints(F.mvpMapPointsBKFs),
        mbFirstConnection(true), mpParent(nullptr), mbNotErase(false),
        mbToBeErased(false), mbBad(false),
        mpORBvocabulary(F.mpORBvocabulary), mpBMap(pBMap), mpBundledKeyFramesDB(pBKFDB),
        mbIsEmpty(false),mbPoseLock(false),mbPoseChanged(false),mbInOutBuffer(false), mbSentOnce(false),
        mLoopQuery(defpair),mMatchQuery(defpair),
        mBALocalForBKFs(defpair),mBAFixedForBKFs(defpair),mBAGlobalForBKFs(defpair),
        mSysState(SysState),mbOmitSending(false),mbLoopCorrected(false),
        mbAck(false),mbFromServer(false),mbUpdatedByServer(false),mCorrected_MM(defpair),mbSendFull(true)     
{
    mId=make_pair(nNextId++,F.mId.second);

    mspComm.insert(pComm);

    mGrid0.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid0[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid0[i][j] = F.mGrid0[i][j];
    }

    mGrid1.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid1[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid1[i][j] = F.mGrid1[i][j];
    }

    vmTi0.resize(cameraNum);
    for(int id = 0; id < cameraNum; id++)
    {
        cv::Mat Ti0 = cv::Mat::eye(4,4,CV_32F);
        for(int x = 0; x < id; x++)
            Ti0 =  F.mvTcamji[x] * Ti0;
        vmTi0[id] = Ti0;
    }
    

    // mGrid2.resize(mnGridCols);
    // for(int i=0; i<mnGridCols;i++)
    // {
    //     mGrid2[i].resize(mnGridRows);
    //     for(int j=0; j<mnGridRows; j++)
    //         mGrid2[i][j] = F.mGrid2[i][j];
    // }

    // mGrid3.resize(mnGridCols);
    // for(int i=0; i<mnGridCols;i++)
    // {
    //     mGrid3[i].resize(mnGridRows);
    //     for(int j=0; j<mnGridRows; j++)
    //         mGrid3[i][j] = F.mGrid3[i][j];
    // }

    SetPose(F.mTcw,false);

    mvbMapPointsLock.resize(N,false);

    if(mUniqueId == 0 && !mId.first == 0) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::KeyFrame(..): mUniqueId not set" << endl;

    mdInsertStamp = ros::Time::now().toNSec();
}

BundledKeyFrames::BundledKeyFrames(ccmslam_msgs::BKF* pMsg, vocptr pVoc, bmapptr pBMap, bdbptr pBKFDB, commptr pComm, eSystemState SysState, size_t UniqueId, g2o::Sim3 mg2oS_wcurmap_wclientmap)
    :mFrameId(defpair),mVisId(-1),
    mbFirstConnection(true),mbNotErase(false),
    mbToBeErased(false),
    mpORBvocabulary(pVoc),mpBMap(pBMap), mpBundledKeyFramesDB(pBKFDB),
    mbIsEmpty(false),mbPoseLock(false),mbPoseChanged(false),mbSentOnce(true),mbInOutBuffer(false),
    mLoopQuery(defpair),mMatchQuery(defpair),
    mBALocalForBKFs(defpair),mBAFixedForBKFs(defpair),mBAGlobalForBKFs(defpair),
    mSysState(SysState),mbOmitSending(false),
    mbLoopCorrected(false),
    mbBad(false),
    mbAck(true),mbFromServer(false),mbUpdatedByServer(false),mCorrected_MM(defpair),mbSendFull(false)
{
    if(pMsg->mbBad)
    {
         cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " Incoming BKFs message: mbBad == true" << endl;
         throw infrastructure_ex();
    }

    mbOmitSending = true;

    if(mSysState == eSystemState::SERVER)
    {
        mUniqueId = UniqueId;
        mbFromServer = false;
    }
    else if(mSysState == eSystemState::CLIENT)
    {
        mUniqueId = pMsg->mUniqueId;
        mbFromServer = true;
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " System Type Not Set" << endl;
        throw infrastructure_ex();
    }

    mspComm.insert(pComm);

    this->WriteMembersFromMessage(pMsg,mg2oS_wcurmap_wclientmap);  //todo

    for(int id = 0; id < cameraNum; id++)
        AssignFeaturesToGrid(id);

    mdInsertStamp = ros::Time::now().toNSec();

    mbOmitSending = false;
}

void BundledKeyFrames::EstablishInitialConnectionsClient()
{
    //this is necessary, because we cannot use shared_from_this() in constructor

    {
        unique_lock<mutex> lock(mMutexFeatures);

        for(int index=0;index<mvpMapPoints.size();++index)
        {
            mpptr pMPi = mvpMapPoints[index];

            if(pMPi)
            {
                pMPi->AddBKFsObservation(shared_from_this(),index, cameraNum);
                pMPi->ComputeDistinctiveDescriptors();
                pMPi->UpdateNormalAndDepthPlus();
            }
            else
            {
                //nullptr -- do not use
            }
        }
    }
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
                mTcp = Tcw*mpParent->GetPoseInverse().clone();
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


void BundledKeyFrames::ReduceMessage(ccmslam_msgs::BKF *pMsgFull, ccmslam_msgs::BKFred *pMsgRed)
{
    pMsgRed->mnId = pMsgFull->mnId;
    pMsgRed->mClientId = pMsgFull->mClientId;
    pMsgRed->mUniqueId = pMsgFull->mUniqueId;
    pMsgRed->mTcpred = pMsgFull->mTcpred;
    pMsgRed->mTcpar = pMsgFull->mTcpar;
    pMsgRed->mpPred_BKfId = pMsgFull->mpPred_BKfId;
    pMsgRed->mpPred_BKfClientId = pMsgFull->mpPred_BKfClientId;
    pMsgRed->mpPar_BKfId = pMsgFull->mpPar_BKfId;
    pMsgRed->mpPar_BKfClientId = pMsgFull->mpPar_BKfClientId;
    pMsgRed->mbBad = pMsgFull->mbBad;

    pMsgRed->mvBDepth = pMsgFull->mvBDepth;
}

void BundledKeyFrames::ConvertToMessage(ccmslam_msgs::BMap &msgBMap, g2o::Sim3 mg2oS_wcurmap_wclientmap, bkfptr pRefBKf, bool bForceUpdateMsg)
{
    unique_lock<mutex> lockOut(mMutexOut);

    if((mbSendFull || mSysState == eSystemState::SERVER) && !bForceUpdateMsg)
    {
        // ccmslam_msgs::KF Msg;

        // //??
        // Msg.mbf = mbf;
        // Msg.mThDepth = mThDepth;

        // Msg.mvuRight.resize(mvuRight.size());
        // for(int idx=0;idx<mvuRight.size();++idx) Msg.mvuRight[idx] = mvuRight[idx];
        // Msg.mvDepth.resize(mvDepth.size());
        // for(int idx=0;idx<mvDepth.size();++idx) Msg.mvDepth[idx] = mvDepth[idx];

        // unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
        // unique_lock<mutex> lock2(mMutexConnections,defer_lock);
        // unique_lock<mutex> lock3(mMutexPose,defer_lock);

        // lock(lock1,lock2,lock3);

        // if(mSysState == eSystemState::SERVER)
        // {         
        //     float s = mg2oS_wcurmap_wclientmap.inverse().scale();
        //     s = 1/s;

        //     cv::Mat Twp = cv::Mat(4,4,5);
        //     cv::Mat Tcp  = cv::Mat(4,4,5);

        //     if(this->mId != pRefKf->mId)
        //     {
        //         Twp = pRefKf->GetPoseInverse();
        //         Tcp = Tcw * Twp;
        //         Tcp.at<float>(0,3) *=(1./s);
        //         Tcp.at<float>(1,3) *=(1./s);
        //         Tcp.at<float>(2,3) *=(1./s);
        //     }
        //     else
        //     {
        //         Twp = cv::Mat::eye(4,4,5);

        //         cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
        //         cv::Mat tcw = Tcw.rowRange(0,3).col(3);
        //         g2o::Sim3 g2oS_c_wm(Converter::toMatrix3d(Rcw),Converter::toVector3d(tcw),1.0); //cam - world map

        //         g2o::Sim3 g2oS_c_wc = g2oS_c_wm*mg2oS_wcurmap_wclientmap;

        //         Eigen::Matrix3d eigR = g2oS_c_wc.rotation().toRotationMatrix();
        //         Eigen::Vector3d eigt = g2oS_c_wc.translation();
        //         float scale = mg2oS_wcurmap_wclientmap.inverse().scale();
        //         scale = 1/scale;
        //         eigt *=(1./scale); //[R t/s;0 1]
        //         cv::Mat T_c_wc = Converter::toCvSE3(eigR,eigt);

        //         Tcp = T_c_wc * Twp;
        //     }

        //     Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KF::_mTcpred_type,float>(Tcp,Msg.mTcpred);

        //     Msg.mpPred_KfId = static_cast<uint16_t>(pRefKf->mId.first);
        //     Msg.mpPred_KfClientId = static_cast<uint8_t>(pRefKf->mId.second);

        //     Msg.mpPar_KfId = KFRANGE;
        //     Msg.mpPar_KfClientId = MAPRANGE;

        //     Msg.mbBad = mbBad;
        //     Msg.bSentOnce = mbSentOnce;

        //     ccptr pCC = mpMap->GetCCPtr(mId.second);
        //     if(pCC->mbOptimized)
        //         Msg.mbPoseChanged = true;
        //     else
        //         Msg.mbPoseChanged = mbPoseChanged;

        //     Msg.mbServerBA = false;
        // }
        // else
        // {
        //     if(this->mId.first == 0)
        //     {
        //         Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KF::_mTcpred_type,float>(Tcw,Msg.mTcpred);
        //     }
        //     else
        //     {
        //         //pose relative to predecessor
        //         kfptr pPred = mpMap->GetPredecessor(shared_from_this());
        //         mTcp = Tcw*pPred->GetPoseInverse();
        //         Msg.mpPred_KfId = static_cast<uint16_t>(pPred->mId.first);
        //         Msg.mpPred_KfClientId = static_cast<uint8_t>(pPred->mId.second);
        //         Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KF::_mTcpred_type,float>(mTcp,Msg.mTcpred);

        //         //pose relative to parent's parent
        //         if(!mpParent)
        //         {
        //             cout << COUTFATAL << ": no parent" << endl;
        //             throw infrastructure_ex();
        //         }
        //         else
        //         {
        //             kfptr pParPar = mpParent->GetParent();
        //             if(!pParPar)
        //                 pParPar = mpParent; //if mpParents has no parent, use mpParent
        //             else if(pParPar->mId == this->mId)
        //             {
        //                 pParPar = mpParent; //if mpParents has no parent, use mpParent
        //             }

        //             mTcp = Tcw*pParPar->GetPoseInverse();
        //             Msg.mpPar_KfId = static_cast<uint16_t>(pParPar->mId.first);
        //             Msg.mpPar_KfClientId = static_cast<uint8_t>(pParPar->mId.second);
        //             Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KF::_mTcpar_type,float>(mTcp,Msg.mTcpar);

        //             Msg.mbServerBA = false;
        //         }
        //     }

        //     //    Msg.mbBad = mbBad;
        //     Msg.mbBad = false; //necessary in case that KF is in out buffer when trimmed from Map
        //     Msg.mbAck = mbAck;

        //     Msg.bSentOnce = mbSentOnce;
        //     mbSentOnce = true;
        //     mbPoseChanged = false;
        //     mbSendFull = false;

        //     ccptr pCC = mpMap->GetCCPtr(this->mId.second);
        //     cv::Mat T_SC = Converter::toCvMat(pCC->mT_SC);
        //     Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KF::_mT_SC_type,float>(T_SC,Msg.mT_SC);
        // }

        // //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // //   ++
        // //   ++
        // // ++++++   Const values
        // //  ++++
        // //   ++

        // Msg.mnGridCols = mnGridCols;
        // Msg.mnGridRows = mnGridRows;
        // Msg.mfGridElementWidthInv = mfGridElementWidthInv;
        // Msg.mfGridElementHeightInv = mfGridElementHeightInv;

        // Msg.fx = fx;
        // Msg.fy = fy;
        // Msg.cx = cx;
        // Msg.cy = cy;
        // Msg.invfx = invfx;
        // Msg.invfy = invfy;

        // Msg.N = static_cast<int16_t>(N);

        // for(int idx=0;idx<mvKeysUn.size();++idx) Msg.mvKeysUn.push_back(Converter::toCvKeyPointMsg(mvKeysUn[idx]));

        // for(int idx=0;idx<mDescriptors.rows;++idx)
        // {
        //     ccmslam_msgs::Descriptor MsgDesc;
        //     for(int idy=0;idy<mDescriptors.cols;++idy)
        //     {
        //         MsgDesc.mDescriptor[idy]=mDescriptors.at<uint8_t>(idx,idy);
        //     }
        //     Msg.mDescriptors.push_back(MsgDesc);
        // }

        // Msg.mnScaleLevels = static_cast<int8_t>(mnScaleLevels);
        // Msg.mfScaleFactor = mfScaleFactor;
        // Msg.mfLogScaleFactor = mfLogScaleFactor;
        // for(int idx=0;idx<mvScaleFactors.size();++idx) Msg.mvScaleFactors[idx] = mvScaleFactors[idx];
        // for(int idx=0;idx<mvLevelSigma2.size();++idx) Msg.mvLevelSigma2[idx] = mvLevelSigma2[idx];
        // for(int idx=0;idx<mvInvLevelSigma2.size();++idx) Msg.mvInvLevelSigma2[idx] = mvInvLevelSigma2[idx];

        // Msg.mnMinX = static_cast<int16_t>(mnMinX);
        // Msg.mnMinY = static_cast<int16_t>(mnMinY);
        // Msg.mnMaxX = static_cast<int16_t>(mnMaxX);
        // Msg.mnMaxY = static_cast<int16_t>(mnMaxY);

        // Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::KF::_mK_type,float>(mK,Msg.mK);

        // //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // //   ++
        // //   ++
        // // ++++++   Map points
        // //  ++++
        // //   ++

        // for(int idx=0;idx<mvpMapPoints.size();++idx)
        // {
        //     if(mvpMapPoints[idx] && !mvpMapPoints[idx]->isBad())
        //     {
        //         Msg.mvpMapPoints_Ids.push_back(static_cast<uint32_t>(mvpMapPoints[idx]->mId.first));
        //         Msg.mvpMapPoints_ClientIds.push_back(static_cast<uint8_t>(mvpMapPoints[idx]->mId.second));
        //         Msg.mvpMapPoints_VectId.push_back(static_cast<uint16_t>(idx));
        //     }
        // }

        // //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // Msg.mnId = static_cast<uint16_t>(mId.first);
        // Msg.mClientId = static_cast<uint8_t>(mId.second);
        // Msg.mUniqueId = static_cast<uint32_t>(mUniqueId);
        // Msg.dTimestamp = mTimeStamp;

        // msgMap.Keyframes.push_back(Msg);
    }
    else
    {
        if(mSysState == eSystemState::SERVER)
        {
            cout << COUTFATAL << " must no be used by server" << endl;
            throw infrastructure_ex();
        }

        ccmslam_msgs::BKFred Msg;

        Msg.mvBDepth.resize(mvBDepth.size());
        for(int index=0;index<mvBDepth.size();++index) Msg.mvBDepth[index] = mvBDepth[index];

        unique_lock<mutex> lockPose(mMutexPose);

            if(mbPoseChanged)
            {
                if(this->mId.first == 0)
                {
                    Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::BKFred::_mTcpred_type,float>(Tcw,Msg.mTcpred);
                }
                else
                {
                    //pose relative to predecessor
                    bkfptr pPred;
                    if(this->mId.second == mpBMap->mBMapId) //native BKF
                        pPred = mpBMap->GetPredecessor(shared_from_this());
                    else
                    {
                        pPred = mpParent;
                    }
                    mTcp = Tcw*pPred->GetPoseInverse();
                    Msg.mpPred_BKfId = static_cast<uint16_t>(pPred->mId.first);
                    Msg.mpPred_BKfClientId = static_cast<uint8_t>(pPred->mId.second);
                    Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::BKFred::_mTcpred_type,float>(mTcp,Msg.mTcpred);

                    //pose relative to parent
                    if(!mpParent)
                    {
                        cout << COUTFATAL << ": no parent" << endl;
                        throw infrastructure_ex();
                    }
                    else
                    {
                        bkfptr pParPar = mpParent->GetParent();
                        if(!pParPar)
                            pParPar = mpParent; //if mpParents has no parent, use mpParent
                        else if(pParPar->mId == this->mId)
                        {
                            pParPar = mpParent; //if mpParents has no parent, use mpParent
                        }

                        mTcp = Tcw*pParPar->GetPoseInverse();
                        Msg.mpPar_BKfId = static_cast<uint16_t>(pParPar->mId.first);
                        Msg.mpPar_BKfClientId = static_cast<uint8_t>(pParPar->mId.second);
                        Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::BKF::_mTcpar_type,float>(mTcp,Msg.mTcpar);
                    }
                }

                Msg.mbServerBA = false;

                mbPoseChanged = false;

                Msg.mbBad = false; //necessary in case that KF is in out buffer when trimmed from Map
                Msg.mnId = static_cast<uint16_t>(mId.first);
                Msg.mClientId = static_cast<uint8_t>(mId.second);
                Msg.mUniqueId = static_cast<uint32_t>(mUniqueId);
                                
                msgBMap.BKFUpdates.push_back(Msg);
            }
            else
            {
                //pose has not changed - do nothing
            }
    }
}

void BundledKeyFrames::UpdateFromMessage(ccmslam_msgs::BKF *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    //if KF already exists, we reduce it to an Update-Msg to use common interfaces

    ccmslam_msgs::BKFred *pMsgRed = new ccmslam_msgs::BKFred();

    this->ReduceMessage(pMsg,pMsgRed);

    this->UpdateFromMessage(pMsgRed,mg2oS_wcurmap_wclientmap);

    delete pMsgRed;
 }

void BundledKeyFrames::UpdateFromMessage(ccmslam_msgs::BKFred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    mbOmitSending = true;

    unique_lock<mutex> lockOut(mMutexOut);
    unique_lock<mutex> lock(mMutexPose);

    if(mSysState == eSystemState::CLIENT)
    {
        bool bSetPose = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

        if(!bSetPose)
        {
            mbOmitSending = false;
            return;
        }

        mbPoseChanged = false;
        mbUpdatedByServer = true;
    }
    // else if(mSysState == eSystemState::SERVER)
    // {
    //     if(!mbPoseLock)
    //     {
    //         if(this->mId.first != 0)
    //         {
    //             bool bSetPose = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

    //             if(!bSetPose)
    //             {
    //                 mbOmitSending = false;
    //                 return;
    //             }
    //         }
    //         else
    //         {
    //             //first KF has no parent

    //             cv::Mat tempTcw = cv::Mat(4,4,5);
    //             Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpred_type,float>(tempTcw,pMsg->mTcpred);
    //             cv::Mat Rcw = tempTcw.rowRange(0,3).colRange(0,3);
    //             cv::Mat tcw = tempTcw.rowRange(0,3).col(3);
    //             g2o::Sim3 g2oS_c_wc(Converter::toMatrix3d(Rcw),Converter::toVector3d(tcw),1.0); //cam - world client

    //             g2o::Sim3 g2oS_c_wm = g2oS_c_wc*(mg2oS_wcurmap_wclientmap.inverse()); //cam client - world map

    //             Eigen::Matrix3d eigR = g2oS_c_wm.rotation().toRotationMatrix();
    //             Eigen::Vector3d eigt = g2oS_c_wm.translation();
    //             float s = static_cast<float>(mg2oS_wcurmap_wclientmap.scale());
    //             s = 1/s;
    //             eigt *=(1./s); //[R t/s;0 1]
    //             cv::Mat T_c_wm = Converter::toCvSE3(eigR,eigt);

    //             this->SetPose(T_c_wm,false,true);
    //         }
    //     }
    // }

    mbOmitSending = false;
}


bool BundledKeyFrames::SetPoseFromMessage(ccmslam_msgs::BKF *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    ccmslam_msgs::BKFred *pMsgRed = new ccmslam_msgs::BKFred();

    this->ReduceMessage(pMsg,pMsgRed);

    bool bReturn = this->SetPoseFromMessage(pMsgRed,mg2oS_wcurmap_wclientmap);

    delete pMsgRed;

    return bReturn;
}

bool BundledKeyFrames::SetPoseFromMessage(ccmslam_msgs::BKFred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    if(mSysState == eSystemState::CLIENT)
    {
        // CLIENT

        idpair MsgRefId = make_pair(pMsg->mpPred_BKfId,pMsg->mpPred_BKfClientId);

        cv::Mat tempTcw = cv::Mat(4,4,5);

        if(MsgRefId == this->mId)
        {
            Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::BKF::_mTcpred_type,float>(tempTcw,pMsg->mTcpred);
        }
        else
        {
            bkfptr pPred = mpBMap->GetBKfsPtr(MsgRefId);
            if(!pPred || pPred->isBad())
            {
                return false;
            }

            cv::Mat Tpw = pPred->GetPose();

            cv::Mat Tcp = cv::Mat(4,4,5);
            Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::BKF::_mTcpred_type,float>(Tcp,pMsg->mTcpred);

            tempTcw = Tcp * Tpw;
        }

        this->SetPose(tempTcw,false,true);
    }
    // else if(mSysState == eSystemState::SERVER)
    // {
    //     // SERVER

    //     if(this->mId.first != 0)
    //     {
    //         cv::Mat tempTcp = cv::Mat(4,4,5);

    //         kfptr pRef = mpMap->GetKfPtr(pMsg->mpPred_KfId,pMsg->mpPred_KfClientId);

    //         if(pRef)
    //         {
    //             Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpred_type,float>(tempTcp,pMsg->mTcpred);
    //         }

    //         if(!pRef)
    //         {
    //             pRef = mpMap->GetKfPtr(pMsg->mpPar_KfId,pMsg->mpPar_KfClientId);

    //             if(pRef)
    //             {
    //                 Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpar_type,float>(tempTcp,pMsg->mTcpar);
    //             }
    //         }

    //         if(!pRef)
    //         {
    //             //check -- maybe we can use erased pointer
    //             pRef = mpMap->GetErasedKfPtr(pMsg->mpPred_KfId,pMsg->mpPred_KfClientId);

    //             if(pRef)
    //             {
    //                 Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpred_type,float>(tempTcp,pMsg->mTcpred);
    //             }
    //         }

    //         if(!pRef)
    //         {
    //             //check -- maybe we can use erased pointer
    //             pRef = mpMap->GetErasedKfPtr(pMsg->mpPar_KfId,pMsg->mpPar_KfClientId);

    //             if(pRef)
    //             {
    //                 Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpar_type,float>(tempTcp,pMsg->mTcpar);
    //             }
    //         }

    //         if(!pRef)
    //         {
    //             //there is no reference pointer available -- ignore this message
    //             return false;
    //         }

    //         float s = static_cast<float>(mg2oS_wcurmap_wclientmap.scale());
    //         s = 1/s; //warn
    //         tempTcp.at<float>(0,3) *=(1./s);
    //         tempTcp.at<float>(1,3) *=(1./s);
    //         tempTcp.at<float>(2,3) *=(1./s);

    //         cv::Mat tempTcw = cv::Mat(4,4,5);

    //         if(!pRef->isBad())
    //         {
    //             cv::Mat Tpw = pRef->GetPose();
    //             tempTcw = tempTcp * Tpw;
    //         }
    //         else
    //         {
    //             kfptr pRefRef = pRef->GetParent();

    //             if(!pRefRef)
    //             {
    //                 cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " parent is bad or not existing" << endl;
    //                 cout << "this: " << this->mId.first << "|" << this->mId.second << endl;
    //                 cout << "pRef: " << pRef->mId.first << "|" << pRef->mId.second << endl;
    //                 cout << "!pRefRef" << endl;
    //                 throw infrastructure_ex();
    //             }

    //             cv::Mat T_p_pp = pRef->mTcp;

    //             while(pRefRef->isBad())
    //             {
    //                 T_p_pp = T_p_pp * pRefRef->mTcp;

    //                 pRefRef = pRefRef->GetParent();

    //                 if(!pRefRef)
    //                 {
    //                     cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " parent is bad or not existing" << endl;
    //                     cout << "this: " << this->mId.first << "|" << this->mId.second << endl;
    //                     cout << "pPred: " << pRef->mId.first << "|" << pRef->mId.second << endl;
    //                     cout << "!pPredPred" << endl;
    //                     throw infrastructure_ex();
    //                 }
    //             }

    //             cv::Mat T_pp_w = pRefRef->GetPose();
    //             tempTcw = tempTcp * T_p_pp * T_pp_w;
    //         }

    //         this->SetPose(tempTcw,false,true);
    //     }
    //     else
    //     {
    //         //first KF has no parent

    //         cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " must not be called for KF 0" << endl;
    //         throw estd::infrastructure_ex();
    //     }
    // }

    return true;
}


void BundledKeyFrames::SetPose(const cv::Mat &Tcw_, bool bLock, bool bIgnorePoseMutex)
{
    if(mbPoseLock && mSysState == eSystemState::CLIENT)
        return;

    {
        if(!bIgnorePoseMutex)
            unique_lock<mutex> lock(mMutexPose);
            //do not acquire mMutexOut, since this might be called from Update-method.

        Tcw_.copyTo(Tcw);
        cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = Tcw.rowRange(0,3).col(3);
        cv::Mat Rwc = Rcw.t();
        Ow = -Rwc*tcw;

        Twc = cv::Mat::eye(4,4,Tcw.type());
        Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
        Ow.copyTo(Twc.rowRange(0,3).col(3));

        vOw.resize(cameraNum);

        for(int id = 0; id < cameraNum; id++)
        {
            if(id == 0)
            {
                vOw[0] = Ow;
                continue;
            }

            cv::Mat tTcami0= cv::Mat::eye(4,4,CV_32F); //T10
            for(int x = 0; x < id; x++)
            {
                tTcami0 = mvTcamji[x]* tTcami0;  //Ti0
            }

            cv::Mat tTiw;
            tTiw = tTcami0 * Tcw;

            cv::Mat tRiw = tTiw.rowRange(0,3).colRange(0,3);
            cv::Mat ttiw = tTiw.rowRange(0,3).col(3);
            cv::Mat ttwi = -tRiw.t()*ttiw;  //mtwi

            vOw[id] = ttwi.clone();
            
        }

        if(mSysState == eSystemState::CLIENT)
            mdPoseTime = ros::Time::now().toNSec();

        if(bLock)
        {
            mbPoseLock = true;
        }
    }

    if(!mbOmitSending && this->IsSent())
    {
        mbPoseChanged = true;
        if(mSysState == CLIENT)  //todo i think it should be commented wwh
             SendMe();
    }
}

void BundledKeyFrames::SetSendFull()
{
    {
        unique_lock<mutex> lock(mMutexOut);
        mbSendFull = true;
    }

    this->SendMe();
}

void BundledKeyFrames::SendMe()
{
    if(mspComm.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m KeyFrame::SendMe(): no Comm ptrs" << endl;
        throw infrastructure_ex();
    }

    if(this->IsSent() && !this->IsInOutBuffer())
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            commptr pComm = *sit;
            pComm->PassBKfstoComm(this->shared_from_this());
        }
    }
}

cv::Mat BundledKeyFrames::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat BundledKeyFrames::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

void BundledKeyFrames::WriteMembersFromMessage(ccmslam_msgs::BKF *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    mbSentOnce=true;
    cameraNum = 2; //todo 2->4;
    mId = make_pair(pMsg->mnId,pMsg->mClientId);
    mTimeStamp = pMsg->dTimestamp;
    mnGridCols = pMsg->mnGridCols;
    mnGridRows=pMsg->mnGridRows;

    mvGridElementWidthInv.resize(pMsg->mvGridElementWidthInv.size());
    mvGridElementHeightInv.resize(pMsg->mvGridElementHeightInv.size());
    for(int idx = 0; idx< pMsg->mvGridElementWidthInv.size(); idx++)
    {
        mvGridElementWidthInv[idx] = pMsg->mvGridElementWidthInv[idx];
        mvGridElementHeightInv[idx] = pMsg->mvGridElementHeightInv[idx];
    }

    vfx.resize(pMsg->vfx.size());
    vfy.resize(pMsg->vfx.size());
    vcx.resize(pMsg->vfx.size());
    vcy.resize(pMsg->vfx.size());
    vinvfx.resize(pMsg->vfx.size());
    vinvfy.resize(pMsg->vfx.size());

    for(int id = 0; id < pMsg->vfx.size(); id++)
    {
        vfx[id] = pMsg->vfx[id];
        vfy[id] = pMsg->vfy[id];
        vcx[id]=pMsg->vcx[id];
        vcy[id]=pMsg->vcy[id];
        vinvfx[id]=pMsg->vinvfx[id];
        vinvfy[id]=pMsg->vinvfy[id];

    }

    N=pMsg->N;

    mvpkeyPointsNum.resize(pMsg->mvpkeyPointsNum.size());
    for(int idx = 0; idx < pMsg->mvpkeyPointsNum.size(); idx++)
    {
        mvpkeyPointsNum[idx] = pMsg->mvpkeyPointsNum[idx];
    }

    vKeyPointsIndexMapPlus.resize(pMsg->vKeyPointsIndexMapPlus.size());
    for(int idx = 0; idx < pMsg->vKeyPointsIndexMapPlus.size(); idx++)
    {
        ccmslam_msgs::KeyPointsIndexMapPlus KeyPointsIndexMapPlusMsg = pMsg->vKeyPointsIndexMapPlus[idx];
        vKeyPointsIndexMapPlus[idx].resize(KeyPointsIndexMapPlusMsg.mKeyPointsIndexMapPlus.size());
        for(int idy = 0; idy < KeyPointsIndexMapPlusMsg.mKeyPointsIndexMapPlus.size(); idy++)
            vKeyPointsIndexMapPlus[idx][idy] = pMsg->vKeyPointsIndexMapPlus[idx].mKeyPointsIndexMapPlus[idy];
    }
    
    mvTcamji.resize(pMsg->mvTcamji.size()); 
    for(int id = 0; id < pMsg->mvTcamji.size(); id++)
    {
        ccmslam_msgs::Tcamji TcamjiMsg = pMsg->mvTcamji[id];
        mvTcamji[id] = cv::Mat::eye(4,4,CV_32F);
        for(int idx = 0; idx < 4; id++)
        {   
            for(int idy = 0; idy < 4;idy++)
            {
                mvTcamji[id].at<float>(idx, idy) = TcamjiMsg.mTcamji[idx*4 + idy];
            }
        }  
    }

    mnScaleLevels=pMsg->mnScaleLevels;
    mfScaleFactor=pMsg->mfScaleFactor;
    mfLogScaleFactor=pMsg->mfLogScaleFactor;

    mvMinX.resize(cameraNum);
    mvMaxX.resize(cameraNum);
    mvMinY.resize(cameraNum);
    mvMaxY.resize(cameraNum);

    for(int id = 0; id < cameraNum; id++)
    {
        mvMinX[id] = pMsg->mvMinX[id];
        vfy[id] = pMsg->vfy[id];
        vcx[id]=pMsg->vcx[id];
        vcy[id]=pMsg->vcy[id];
        vinvfx[id]=pMsg->vinvfx[id];
        vinvfy[id]=pMsg->vinvfy[id];

    }

    mvKeysMultipleUn.resize(pMsg->mvKeysMultipleUn.size());  //4
    for(int id =0 ; id < pMsg->mvKeysMultipleUn.size(); id++)
    {
        ccmslam_msgs::VCvKeyPoint VCvKeyPointMsg = pMsg->mvKeysMultipleUn[id];
        mvKeysMultipleUn[id].resize(VCvKeyPointMsg.mvKeysUn.size()); //100
        for(int idi=0;idi<VCvKeyPointMsg.mvKeysUn.size();++idi)
        {
            mvKeysMultipleUn[id][idi] = Converter::fromCvKeyPointMsg(VCvKeyPointMsg.mvKeysUn[idi]);
        }
    }

    mvpDescriptors.resize(pMsg->mvpDescriptors.size());  //4
    for(int id =0 ; id < pMsg->mvpDescriptors.size(); id++)
    {
        
        ccmslam_msgs::VDescriptor VDescriptorMsg = pMsg->mvpDescriptors[id];  //mat n*32
        int iBoundX = static_cast<int>(VDescriptorMsg.mDescriptors.size()); // n
        ccmslam_msgs::Descriptor TempDesc = VDescriptorMsg.mDescriptors[0]; 
        int iBoundY = static_cast<int>(TempDesc.mDescriptor.size());  //32
        mvpDescriptors[id] = cv::Mat(iBoundX,iBoundY,0);

        //mat
        for(int idx=0;idx<iBoundX;++idx)
        {
            ccmslam_msgs::Descriptor DescMsg = VDescriptorMsg.mDescriptors[idx];
            for(int idy=0;idy<iBoundY;++idy)  //32
            {
                mvpDescriptors[id].at<uint8_t>(idx,idy)=DescMsg.mDescriptor[idy];
            }
        }
    }

    ccmslam_msgs::Descriptor TempDesc = pMsg->mDescriptors[0]; //Mat
    int iBoundY = static_cast<int>(TempDesc.mDescriptor.size()); //32
    int iBoundX = static_cast<int>(pMsg->mDescriptors.size()); //n
    mDescriptors = cv::Mat(iBoundX,iBoundY,0);

    for(int idx=0;idx<pMsg->mDescriptors.size();++idx)
    {
        ccmslam_msgs::Descriptor DescMsg = pMsg->mDescriptors[idx];
        for(int idy=0;idy<iBoundY;++idy)
        {
            mDescriptors.at<uint8_t>(idx,idy)=DescMsg.mDescriptor[idy];
        }
    }


    for(int idx=0;idx<mnScaleLevels;++idx)
        mvScaleFactors.push_back(pMsg->mvScaleFactors[idx]);

    for(int idx=0;idx<mnScaleLevels;++idx)
        mvLevelSigma2.push_back(pMsg->mvLevelSigma2[idx]);

    for(int idx=0;idx<mnScaleLevels;++idx)
        mvInvLevelSigma2.push_back(pMsg->mvInvLevelSigma2[idx]);

  
    mvBDepth.resize(pMsg->mvBDepth.size());
    for(int idx=0;idx<pMsg->mvBDepth.size();++idx)
        mvBDepth[idx] = pMsg->mvBDepth[idx];


    mvpK.resize(pMsg->mvpK.size());
    for(int id = 0; id <pMsg->mvpK.size(); id++)
    {
        mvpK[id] = cv::Mat(3,3,5);
        ccmslam_msgs::Vfloat32  Vfloat32Msg = pMsg->mvpK[id]; //mat 3*3

        Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::Vfloat32::_mK_type,float>(mvpK[id],Vfloat32Msg.mK);
    }

    

    if(!mBowVec.empty() || !mFeatVec.empty())
    {
        cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " !mBowVec.empty() || !mFeatVec.empty()" << endl;
    }

    vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
    // Feature vector associate features with nodes in the 4th level (from leaves up)
    // We assume the vocabulary tree has 6 levels, change the 4 otherwise
    mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);

    //find mappoints

    mvpMapPoints.resize(N,nullptr);
    mvbMapPointsLock.resize(N,false);

    for(int index=0;index<pMsg->mvpMapPoints_Ids.size();++index)
    {
        size_t FeatId = pMsg->mvpMapPoints_VectId[index];

        mpptr pMP = mpBMap->GetMpPtr(pMsg->mvpMapPoints_Ids[index],pMsg->mvpMapPoints_ClientIds[index]);

        if(pMP)
        {
            mvpMapPoints[FeatId] = pMP;
        }
        else
        {
            //if MP not in Map, we ignore it. Might be deleted, or comes in later and is then (hopefully) added to this KF
        }
    }

    Tcw = cv::Mat(4,4,5);
    Twc = cv::Mat(4,4,5);
    Ow = cv::Mat(3,1,5);
    mTcp = cv::Mat(4,4,5);

    if(mSysState == eSystemState::CLIENT)
    {
        // CLIENT

        bool bSetPose = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

        if(!bSetPose)
        {
            mvpMapPoints.clear();
            mbBad = true;
            return;
        }
    }
    // else if(mSysState == eSystemState::SERVER)
    // {
    //     // SERVER

    //     cv::Mat T_SC = cv::Mat(4,4,5);
    //     Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mT_SC_type,float>(T_SC,pMsg->mT_SC);
    //     mT_SC = Converter::toMatrix4d(T_SC);

    //     if(this->mId.first != 0)
    //     {
    //         bool bSetPose = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

    //         if(!bSetPose)
    //         {
    //             mvpMapPoints.clear();
    //             mbBad = true;
    //             return;
    //         }
    //     }
    //     else
    //     {
    //         //first KF has no parent

    //         cv::Mat tempTcw = cv::Mat(4,4,5);
    //         Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::KF::_mTcpred_type,float>(tempTcw,pMsg->mTcpred);

    //         float s = static_cast<float>(mg2oS_wcurmap_wclientmap.scale());
    //         tempTcw.at<float>(0,3) /=(1./s);
    //         tempTcw.at<float>(1,3) /=(1./s);
    //         tempTcw.at<float>(2,3) /=(1./s);

    //         this->SetPose(tempTcw,false);
    //     }
    // }
}

void BundledKeyFrames::AssignFeaturesToGrid(const int &cameraId)
{
    int nReserve = 0.5f*mvpkeyPointsNum[cameraId]/(mnGridCols*mnGridRows);
    if(cameraId == 0)
    {
        mGrid0.resize(mnGridCols);
        for(unsigned int i=0; i<mnGridCols;i++)
        {
            mGrid0[i].resize(mnGridRows);
            for (unsigned int j=0; j<mnGridRows;j++)
                mGrid0[i][j].reserve(nReserve);
        }
        for(int i=0;i<mvpkeyPointsNum[cameraId];i++)
        {
            const cv::KeyPoint &kp = mvKeysMultipleUn[cameraId][i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY, cameraId))
                mGrid0[nGridPosX][nGridPosY].push_back(i);
        }
    }
    else if(cameraId == 1)
    {
        mGrid1.resize(mnGridCols);
        for(unsigned int i=0; i<mnGridCols;i++)
        {
            mGrid1[i].resize(mnGridRows);
            for (unsigned int j=0; j<mnGridRows;j++)
                mGrid1[i][j].reserve(nReserve);
        }
        for(int i=0;i<mvpkeyPointsNum[cameraId];i++)
        {
            const cv::KeyPoint &kp = mvKeysMultipleUn[cameraId][i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY, cameraId))
                mGrid1[nGridPosX][nGridPosY].push_back(i);
        }
    }
    else if(cameraId == 2)
    {
        mGrid2.resize(mnGridCols);
        for(unsigned int i=0; i<mnGridCols;i++)
        {
            mGrid2[i].resize(mnGridRows);
            for (unsigned int j=0; j<mnGridRows;j++)
                mGrid2[i][j].reserve(nReserve);
        }
        for(int i=0;i<mvpkeyPointsNum[cameraId];i++)
        {
            const cv::KeyPoint &kp = mvKeysMultipleUn[cameraId][i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY, cameraId))
                mGrid2[nGridPosX][nGridPosY].push_back(i);
        }
    }
    else
    {
       mGrid3.resize(mnGridCols);
        for(unsigned int i=0; i<mnGridCols;i++)
        {
            mGrid3[i].resize(mnGridRows);
            for (unsigned int j=0; j<mnGridRows;j++)
                mGrid3[i][j].reserve(nReserve);
        }
        for(int i=0;i<mvpkeyPointsNum[cameraId];i++)
        {
            const cv::KeyPoint &kp = mvKeysMultipleUn[cameraId][i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY, cameraId))
                mGrid3[nGridPosX][nGridPosY].push_back(i);
        }
    }
}

bool BundledKeyFrames::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY, const int &cameraId)
{

    posX = round((kp.pt.x - mvMinX[cameraId])*mvGridElementWidthInv[cameraId]);
    posY = round((kp.pt.y - mvMinY[cameraId])*mvGridElementHeightInv[cameraId]);


    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=mnGridCols || posY<0 || posY>=mnGridRows)
        return false;

    return true;
}

cv::Mat BundledKeyFrames::GetCameraCenter(const int &cameraId)
{
    unique_lock<mutex> lock(mMutexPose);
    return vOw[cameraId].clone();
}

bool BundledKeyFrames::IsInImage(const float &x, const float &y, const int &cameraId) const
{
    return (x>=mvMinX[cameraId] && x<mvMaxX[cameraId] && y>=mvMinY[cameraId] && y<mvMaxY[cameraId]);
}


vector<size_t> BundledKeyFrames::GetFeaturesInArea(const float &x, const float &y, const float &r, const int &cameraId) const
{

    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mvMinX[cameraId]-r)*mvGridElementWidthInv[cameraId]));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mvMinX[cameraId]+r)*mvGridElementWidthInv[cameraId]));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mvMinY[cameraId]-r)*mvGridElementHeightInv[cameraId]));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mvMinY[cameraId]+r)*mvGridElementHeightInv[cameraId]));
    if(nMaxCellY<0)
        return vIndices;


    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            if(cameraId == 0)
            {
                const vector<size_t> vCell = mGrid0[ix][iy];
                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysMultipleUn[cameraId][vCell[j]];
                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;

                    if(fabs(distx)<r && fabs(disty)<r)
                        vIndices.push_back(vCell[j]);
                }
            }
            else if(cameraId == 1)
            {
                const vector<size_t> vCell = mGrid1[ix][iy];
                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysMultipleUn[cameraId][vCell[j]];
                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;

                    if(fabs(distx)<r && fabs(disty)<r)
                        vIndices.push_back(vCell[j]);
                }
            }
            else if(cameraId == 2)
            {
                const vector<size_t> vCell = mGrid2[ix][iy];
                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysMultipleUn[cameraId][vCell[j]];
                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;

                    if(fabs(distx)<r && fabs(disty)<r)
                        vIndices.push_back(vCell[j]);
                }
            }
            else
            {
                const vector<size_t> vCell = mGrid3[ix][iy];
                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysMultipleUn[cameraId][vCell[j]];
                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;

                    if(fabs(distx)<r && fabs(disty)<r)
                        vIndices.push_back(vCell[j]);
                }
                
            }
              
            
        }
    }

    return vIndices;
}

set<BundledKeyFrames::mpptr> BundledKeyFrames::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<mpptr> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        mpptr pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

}