#include <cslam/MapPoint.h>

namespace cslam {

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

// MapPoint::MapPoint(const cv::Mat &Pos, kfptr pRefKF, mapptr pMap, size_t ClientId, commptr pComm, eSystemState SysState, size_t UniqueId)
//     : mFirstKfId(pRefKF->mId), mFirstFrame(pRefKF->mFrameId), mUniqueId(UniqueId),
//       nObs(0), mTrackReferenceForFrame(defpair),mLastFrameSeen(defpair),
//       mpReplaced(nullptr), mfMinDistance(0), mfMaxDistance(0),
//       mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
//       mpMap(pMap),
//       mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(false),mbInOutBuffer(false),
//       mLoopPointForKF_LC(defpair), mCorrectedByKF_LC(defpair),mCorrectedReference_LC(-1),
//       mLoopPointForKF_MM(defpair), mCorrectedByKF_MM(defpair),mCorrectedReference_MM(-1),
//       mBAGlobalForKF(defpair),mBALocalForKF(defpair),
//       mFuseCandidateForKF(defpair),
//       mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
//       mbLoopCorrected(false),mbMultiUse(false),
//       mbAck(false),mbFromServer(false),mbUpdatedByServer(false),mInsertedWithKF(-1),
//       mMaxObsKFId(0),mbSendFull(true)
// {
//     mspComm.insert(pComm);

//     Pos.copyTo(mWorldPos);

//     if(mpRefKF)
//     {
//         cv::Mat T_cref_w =  mpRefKF->GetPose();

//         cv::Mat Rcw = T_cref_w.rowRange(0,3).colRange(0,3);
//         cv::Mat tcw = T_cref_w.rowRange(0,3).col(3);

//         mRefPos = Rcw * mWorldPos + tcw;
//     }
//     else
//     {
//         cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": MP has no parent" << endl;
//         throw infrastructure_ex();
//     }

//     mNormalVector = cv::Mat::zeros(3,1,CV_32F);

//     // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
//     while(!mpMap->LockPointCreation()){usleep(params::timings::miLockSleep);}
//     mId=make_pair(nNextId++,ClientId);
//     mpMap->UnLockPointCreation();

//     if(mUniqueId == 0 && !mId.first == 0) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::MapPoint(..): mUniqueId not set" << endl;
// }

MapPoint::MapPoint(const cv::Mat &Pos, bkfptr pRefBKFs, bmapptr pBMap, size_t ClientId, commptr pComm, eSystemState SysState, size_t UniqueId)
    :mFirstBKfsId(pRefBKFs->mId),mFirstFrame(pRefBKFs->mFrameId), mUniqueId(UniqueId), 
    nObs(0), mTrackReferenceForFrame(defpair),mLastFrameSeen(defpair), 
    mBAGlobalForBKFs(defpair), mBALocalForBKFs(defpair), mFuseCandidateForBKFs(defpair), 
    mpRefBKFs(pRefBKFs), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(nullptr), mfMinDistance(0), mfMaxDistance(0), mpBMap(pBMap),
    mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(false),mbInOutBuffer(false),
    mLoopPointForBKF_LC(defpair), mCorrectedByBKF_LC(defpair),mCorrectedReference_LC(-1),
    mLoopPointForBKF_MM(defpair), mCorrectedByBKF_MM(defpair),mCorrectedReference_MM(-1),
    mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
    mbLoopCorrected(false),mbMultiUse(false),
    mbAck(false),mbFromServer(false),mbUpdatedByServer(false),mInsertedWithBKFs(-1),
    mMaxObsBKFsId(0),mbSendFull(true)

{
    mspComm.insert(pComm);

    Pos.copyTo(mWorldPos);

    if(pRefBKFs)
    {
        cv::Mat T_cref_w =  pRefBKFs->GetPose();

        cv::Mat Rcw = T_cref_w.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = T_cref_w.rowRange(0,3).col(3);

        mRefPos = Rcw * mWorldPos + tcw;
    }
    else
    {
        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": MP has no parent" << endl;
        throw infrastructure_ex();
    }

    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    while(!mpBMap->LockPointCreation()){usleep(params::timings::miLockSleep);}
    mId=make_pair(nNextId++,ClientId);
    mpBMap->UnLockPointCreation();

    if(mUniqueId == 0 && !mId.first == 0) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::MapPoint(..): mUniqueId not set" << endl;

}

MapPoint::MapPoint(const cv::Mat &Pos, bmapptr pBMap, frameptr pFrame, const int &cameraId, const int &indexF, size_t ClientId)
    :mFirstBKfsId(pFrame->mpReferenceBKFs->mId), mFirstFrame(pFrame->mId), nObs(0), mTrackReferenceForFrame(defpair), mLastFrameSeen(defpair),
    mBALocalForBKFs(defpair), mFuseCandidateForBKFs(defpair),mLoopPointForBKF_LC(defpair), mCorrectedByBKF_LC(defpair),
    mCorrectedReference_LC(-1), mBAGlobalForBKFs(defpair), mpRefBKFs(nullptr), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(nullptr), mpBMap(pBMap)
{

    Pos.copyTo(mWorldPos);
    cv::Mat twi = pFrame->GetCameraCenter(cameraId); //twi
    mNormalVector = mWorldPos - twi;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - twi;
    const float dist = cv::norm(PC);
    int idxF = pFrame->vKeyPointsIndexMapPlus[indexF][cameraId];
    const int level = pFrame->mvKeysMultipleUn[cameraId][idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mvpDescriptors[cameraId].row(indexF).copyTo(mDescriptor);  //maybe change to average descriptor
    //pFrame->mJointDescriptors.row(indexF).copyTo(mDescriptor);
    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    while(!mpBMap->LockPointCreation()){usleep(params::timings::miLockSleep);}
    mId=make_pair(nNextId++,ClientId);
    mpBMap->UnLockPointCreation();
    
}

MapPoint::MapPoint(ccmslam_msgs::MP *pMsg, bmapptr pBMap, commptr pComm, eSystemState SysState, size_t UniqueId, g2o::Sim3 g2oS_wm_wc)
    : nObs(0),mpReplaced(nullptr),mpBMap(pBMap),
      mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(true),mbInOutBuffer(false),
      mLoopPointForBKF_LC(defpair), mCorrectedByBKF_LC(defpair),mCorrectedReference_LC(defid),
      mLoopPointForBKF_MM(defpair), mCorrectedByBKF_MM(defpair),mCorrectedReference_MM(defid),
      mBAGlobalForBKFs(defpair), mBALocalForBKFs(defpair), mFuseCandidateForBKFs(defpair), 
      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
      mbLoopCorrected(false),mbMultiUse(false),
      mbBad(false),mbAck(true),mbUpdatedByServer(false),mInsertedWithBKFs(-1),mMaxObsBKFsId(0),mbSendFull(false)
{
    if(pMsg->mbBad)
    {
         cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "Incoming KF message: mbBad == true" << endl;
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

    this->WriteMembersFromMessage(pMsg,g2oS_wm_wc);

    mbOmitSending = false;

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Call EstablishInitialConnectionsServer()
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}


MapPoint::MapPoint(ccmslam_msgs::MP *pMsg, mapptr pMap, commptr pComm, eSystemState SysState, size_t UniqueId, g2o::Sim3 g2oS_wm_wc)
    : nObs(0),mpReplaced(nullptr),mpMap(pMap),
      mbIsEmpty(false), mbPoseLock(false),mbPoseChanged(false), mbSentOnce(true),mbInOutBuffer(false),
      mLoopPointForKF_LC(defpair), mCorrectedByKF_LC(defpair),mCorrectedReference_LC(defid),
      mLoopPointForKF_MM(defpair), mCorrectedByKF_MM(defpair),mCorrectedReference_MM(defid),
      mBAGlobalForKF(defpair),mBALocalForKF(defpair),
      mFuseCandidateForKF(defpair),
      mSysState(SysState),mbDoNotReplace(false),mbOmitSending(false),
      mbLoopCorrected(false),mbMultiUse(false),
      mbBad(false),mbAck(true),mbUpdatedByServer(false),mInsertedWithKF(-1),mMaxObsKFId(0),mbSendFull(false)
{
    if(pMsg->mbBad)
    {
         cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << "Incoming KF message: mbBad == true" << endl;
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

    this->WriteMembersFromMessage(pMsg,g2oS_wm_wc);

    mbOmitSending = false;

    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Call EstablishInitialConnectionsServer()
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}




void MapPoint::EstablishInitialConnectionsServer()
{
    //this is necessary, because we cannot use shared_from_this() in constructor

    #ifdef LOGGING
    ccptr pCC;
    if(this->mSysState == SERVER)
    {
        pCC = mpBMap->GetCCPtr(this->mId.second);
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
    }
    #endif

    {
        #ifdef TRACELOCK
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
        #endif

        unique_lock<mutex> lock1(mMutexFeatures);

        #ifdef TRACELOCK
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
        #endif

        for(map<bkfptr,size_t>::iterator mit = mObservationsBKFs.begin();mit != mObservationsBKFs.end();)
        {
            bkfptr pBKFi = mit->first;
            size_t index = mit->second;

            if(pBKFi)
            {
                #ifdef TRACELOCK
                pCC->mpLogger->SetMP(__LINE__,this->mId.second);
                #endif

                mpptr pMP = pBKFi->GetMapPoint(index);

                #ifdef TRACELOCK
                pCC->mpLogger->SetMP(__LINE__,this->mId.second);
                #endif

                if(pMP)
                {
                    //there is already an associated MP to this feature

                    #ifdef TRACELOCK
                    pCC->mpLogger->SetMP(__LINE__,this->mId.second);
                    #endif

                    if(shared_from_this()->mId == pMP->mId)
                    {
                        //But it's this MP, and we're calling the constructor -- thats inconsistent
                        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << "Constructing MP & is already associated to BKF" << endl;
                        cout << "BKF-ID: " << pBKFi->mId.first << "|" << pBKFi->mId.second << " --- this MP-ID: " << this->mId.first << "|" << this->mId.second << " --- comp. MP-ID: " << pMP->mId.first << "|" << pMP->mId.second << endl;
                        throw infrastructure_ex();
                    }
                    else if(!pBKFi->IsMpLocked(index))
                    {
                        pBKFi->ReplaceMapPointMatch(index,shared_from_this(),false);
                        pMP->EraseBKFsObservation(pBKFi, pBKFi->cameraNum);
                    }
                    else
                    {
                        //its locked - cannot replace
                        //delete KF association
                        mit = mObservationsBKFs.erase(mit);
                        nObs--;
                        continue;
                    }

                    #ifdef TRACELOCK
                    pCC->mpLogger->SetMP(__LINE__,this->mId.second);
                    #endif
                }
                else
                {
                    //nothing associated to this feature, we can add
                    pBKFi->AddMapPoint(shared_from_this(),index,false);
                }
            }
            else
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " pBKFi is NULLPTR" << endl;
            }

            ++mit;
        }

        #ifdef TRACELOCK
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
        #endif

        if(nObs == 0)
        {
            mbBad = true;
            #ifdef LOGGING
            pCC->mpLogger->SetMP(__LINE__,this->mId.second);
            #endif
            return;
        }

        mpRefBKFs = mObservationsBKFs.begin()->first; //should not be bad, we checked that when we added the KF

        if(mpRefBKFs->isBad())
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mpRefBKFs is BAD" << endl;
        }
    }

    #ifdef TRACELOCK
    pCC->mpLogger->SetMP(__LINE__,this->mId.second);
    #endif

    this->UpdateNormalAndDepthPlus();

    #ifdef LOGGING
    pCC->mpLogger->SetMP(__LINE__,this->mId.second);
    #endif
}

void MapPoint::EstablishInitialConnectionsClient()
{
    {
        unique_lock<mutex> lock1(mMutexFeatures);

        for(map<bkfptr,size_t>::iterator mit = mObservationsBKFs.begin();mit != mObservationsBKFs.end();)
        {
            bkfptr pBKFsi = mit->first;
            size_t index = mit->second;

            if(pBKFsi)
            {
                mpptr pMP = pBKFsi->GetMapPoint(index);
                if(pMP)
                {
                    //there is already an associated MP to this feature -- replace or discard? ToDo: find best solution

                    if(shared_from_this()->mId == pMP->mId)
                    {
                        //But it's this MP, and we're calling the constructor -- thats inconsistent
                        // cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << "Constructing MP & is already associated to KF" << endl;
                        // cout << "KF-ID: " << pKFi->mId.first << "|" << pKFi->mId.second << " --- this MP-ID: " << this->mId.first << "|" << this->mId.second << " --- comp. MP-ID: " << pMP->mId.first << "|" << pMP->mId.second << endl;
                        // cout << "(pMP = shared_from_this()): " << (int)(pMP == shared_from_this()) << std::endl;

                        if(pMP = shared_from_this())
                        {
                            ++mit;
                            continue;
                        }
                        else
                            mbBad = true;
                            return;
                    }

                        //delete BKF association
                        mit = mObservationsBKFs.erase(mit);
                        nObs--;

                        continue;
                }
                else
                {
                    //nothing assocaited to this feature, we can add
                    pBKFsi->AddMapPoint(shared_from_this(),index,false);
                }
            }
            else
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " pKFi is NULLPTR" << endl;
            }

            ++mit;
        }

        if(nObs == 0)
        {
            mbBad = true; //we need to do that, since w/o Obs, no ref ptr can exist
            return;
        }

        mpRefBKFs = mObservationsBKFs.begin()->first; //should not be bad, we checked taht when we added the KF

        if(mpRefBKFs->isBad())
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": mpRefKF is BAD" << endl;
        }
    }

    this->UpdateNormalAndDepthPlus();

    mMaxObsBKFsId = 0;
    for(std::map<bkfptr,size_t>::iterator mit = mObservationsBKFs.begin();mit!=mObservationsBKFs.end();++mit)
    {
        bkfptr pBKFs = mit->first;
        if(pBKFs->mId.first > mMaxObsBKFsId && pBKFs->mId.second == mpBMap->mBMapId)
            mMaxObsBKFsId = pBKFs->mId.first;
    }
}

void MapPoint::SetWorldPos(const cv::Mat &Pos, bool bLock, bool bIgnorePosMutex)
{
    if(mbPoseLock && mSysState == eSystemState::CLIENT)
        return;

    {
        unique_lock<mutex> lock2(mGlobalMutex);
        if(!bIgnorePosMutex)
            unique_lock<mutex> lock(mMutexPos);

        Pos.copyTo(mWorldPos);

        if(bLock)
        {
            mbPoseLock = true;
        }
    }

    //unique_lock<mutex> lockMap(mMutexOut); //no need for mMutexOut, since ConvertToMsg needs Pos mutex
    if(!mbOmitSending && this->IsSent())
    {
        mbPoseChanged = true;
        if(mSysState == CLIENT)
            SendMe();
    }
}

void MapPoint::SetWorldPosFromRef(const Mat &Pos, bool bLock, kfptr pRefKf)
{
//    unique_lock<mutex> lock2(mGlobalMutex); //would cause deadlock when called from "UpdateFromMessage"
//    unique_lock<mutex> lock(mMutexPos);  //would cause deadlock when called from "UpdateFromMessage"

    Pos.copyTo(mRefPos);

    if(pRefKf)
    {
        cv::Mat T_w_cref =  pRefKf->GetPoseInverse();

        cv::Mat Rwc = T_w_cref.rowRange(0,3).colRange(0,3);
        cv::Mat twc = T_w_cref.rowRange(0,3).col(3);

        mWorldPos = Rwc * mRefPos + twc;
    }
    else
    {
        cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << __LINE__ << ": MP has no parent" << endl;
        throw infrastructure_ex();
    }

    if(bLock)
    {
        mbPoseLock = true;
    }
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

MapPoint::kfptr MapPoint::GetReferenceKeyFrame()
{
     unique_lock<mutex> lock(mMutexFeatures);
     return mpRefKF;
}

MapPoint::bkfptr MapPoint::GetReferenceBundledKeyFrame()
{
     unique_lock<mutex> lock(mMutexFeatures);
     return mpRefBKFs;
}


void MapPoint::SetReferenceKeyFrame(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mpRefKF = pKF;
}

void MapPoint::SetReferenceBundledKeyFrame(bkfptr pBKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mpRefBKFs = pBKF;
}

void MapPoint::AddObservation(kfptr pKF, size_t idx, bool bLock)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;

    if(mObservationsLock.count(pKF) && mSysState == eSystemState::CLIENT)
        return;

    mObservations[pKF]=idx;
    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;

    if(mSysState == eSystemState::CLIENT)
    {
        if(pKF->mId.first > mMaxObsKFId && pKF->mId.second == mpMap->mMapId)
            mMaxObsKFId = pKF->mId.first;
    }

    if(bLock)
    {
        mObservationsLock[pKF] = true;
    }
}

void MapPoint::AddBKFsObservation(bkfptr pBKFs, size_t index, size_t cameraNum, bool bLock)
{
     unique_lock<mutex> lock(mMutexFeatures);
    if(mObservationsBKFs.count(pBKFs))
        return;

    if(mObservationsBKFsLock.count(pBKFs) && mSysState == eSystemState::CLIENT)
        return;

    mObservationsBKFs[pBKFs]=index;

    vector<int> pair_index =pBKFs->vKeyPointsIndexMapPlus[index];

    for(int i = 0; i < cameraNum; i++)
    {
        if(pair_index[i]>=0)
            nObs++;
    }

    if(mSysState == eSystemState::CLIENT)
    {
        if(pBKFs->mId.first > mMaxObsKFId && pBKFs->mId.second == mpBMap->mBMapId)
            mMaxObsKFId = pBKFs->mId.first;
    }

    if(bLock)
    {
        mObservationsBKFsLock[pBKFs] = true;
    }
}

void MapPoint::EraseObservation(kfptr pKF, bool bLock, bool bSuppressMapAction)
{
    if(mObservationsLock.count(pKF) && mSysState == eSystemState::CLIENT)
        return;

    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {   
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else        
                nObs--;

            mObservations.erase(pKF);

            if(mSysState == eSystemState::CLIENT)
            {
                if(pKF->mId.first == mMaxObsKFId && pKF->mId.second == mpMap->mMapId)
                {
                    mMaxObsKFId = 0;
                    for(std::map<kfptr,size_t>::iterator mit = mObservations.begin();mit!=mObservations.end();++mit)
                    {
                        if(pKF->mId.first > mMaxObsKFId && pKF->mId.second == mpMap->mMapId)
                            mMaxObsKFId = pKF->mId.first;
                    }
                }
            }

            if(bLock)
            {
                mObservationsLock[pKF] = true;
            }

            if(mpRefKF==pKF)
            {
                if(nObs > 0)
                {
                    mpRefKF = nullptr;
                    std::map<kfptr,size_t>::const_iterator mitRef = mObservations.begin();
                    while(!mpRefKF)
                    {
                        if(mitRef == mObservations.end()) break;
                        if(!(mitRef->first->isBad()))
                            mpRefKF=mitRef->first;
                        else
                            ++mitRef;
                    }
                }
                else
                    mpRefKF=nullptr;
            }

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag(bSuppressMapAction);

    if(!mpRefKF)
    {
        if(!mbBad)
        {
            SetBadFlag(bSuppressMapAction);
        }
    }
}

void MapPoint::EraseBKFsObservation(bkfptr pBKFs, size_t cameraNum, bool bLock, bool bSuppressMapAction)
{
    if(mObservationsBKFsLock.count(pBKFs) && mSysState == eSystemState::CLIENT)
        return;

    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservationsBKFs.count(pBKFs))
        {   
            int index = mObservationsBKFs[pBKFs];

            for(int k = 0; k < cameraNum; k++)
            {
                if(pBKFs->vKeyPointsIndexMapPlus[index][k]>=0)
                    nObs--;
            }
            mObservationsBKFs.erase(pBKFs);

            if(mSysState == eSystemState::CLIENT)
            {
                if(pBKFs->mId.first == mMaxObsBKFsId && pBKFs->mId.second == mpBMap->mBMapId)
                {
                    mMaxObsBKFsId = 0;
                    for(std::map<bkfptr,size_t>::iterator mit = mObservationsBKFs.begin();mit!=mObservationsBKFs.end();++mit)
                    {
                        if(pBKFs->mId.first > mMaxObsBKFsId && pBKFs->mId.second == mpBMap->mBMapId)
                            mMaxObsBKFsId = pBKFs->mId.first;
                    }
                }
            }

            if(bLock)
            {
                mObservationsBKFsLock[pBKFs] = true;
            }

            if(mpRefBKFs==pBKFs)
            {
                if(nObs > 0)
                {
                    mpRefBKFs = nullptr;
                    std::map<bkfptr,size_t>::const_iterator mitBRef = mObservationsBKFs.begin();
                    while(!mpRefBKFs)
                    {
                        if(mitBRef == mObservationsBKFs.end()) break;
                        if(!(mitBRef->first->isBad()))
                            mpRefBKFs=mitBRef->first;
                        else
                            ++mitBRef;
                    }
                }
                else
                    mpRefBKFs=nullptr;
            }

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlagBKFs(bSuppressMapAction);  //in order to delete this mappoint, we consider this mappoint is bad one.

    if(!mpRefBKFs)
    {
        if(!mbBad)
        {
            SetBadFlagBKFs(bSuppressMapAction);
        }
    }
}

map<MapPoint::kfptr, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

map<MapPoint::bkfptr, size_t> MapPoint::GetBKFsObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservationsBKFs;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag(bool bSuppressMapAction)
{
    #ifdef LOGGING
    ccptr pCC;
    if(this->mSysState == SERVER)
    {
        pCC = mpMap->GetCCPtr(this->mId.second);
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
    }
    #endif

    {
        if(mbBad)
        {
            #ifdef LOGGING
            if(this->mSysState == SERVER)
                pCC->mpLogger->SetMP(__LINE__,this->mId.second);
            #endif
            return;
        }
    }

    map<kfptr,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }

    for(map<kfptr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        kfptr pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    if(!this->IsSent())
    {
        this->EraseInOutBuffer();
    }

    if(!bSuppressMapAction)
        mpMap->EraseMapPoint(this->shared_from_this());
    else
        mpMap->mspMPsToErase.insert(this->shared_from_this());

    #ifdef LOGGING
    if(this->mSysState == SERVER)
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
    #endif
}

void MapPoint::SetBadFlagBKFs(bool bSuppressMapAction)
{
        #ifdef LOGGING
    ccptr pCC;
    if(this->mSysState == SERVER)
    {
        pCC = mpBMap->GetCCPtr(this->mId.second);
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
    }
    #endif

    {
        if(mbBad)
        {
            #ifdef LOGGING
            if(this->mSysState == SERVER)
                pCC->mpLogger->SetMP(__LINE__,this->mId.second);
            #endif
            return;
        }
    }

    map<bkfptr,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservationsBKFs;
        mObservationsBKFs.clear();
    }

    for(map<bkfptr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        bkfptr pBKFs = mit->first;
        pBKFs->EraseMapPointMatch(mit->second);
    }

    if(!this->IsSent())
    {
        this->EraseInOutBuffer();
    }

    if(!bSuppressMapAction)
        mpBMap->EraseMapPoint(this->shared_from_this());
    else
        mpBMap->mspMPsToErase.insert(this->shared_from_this());

    #ifdef LOGGING
    if(this->mSysState == SERVER)
        pCC->mpLogger->SetMP(__LINE__,this->mId.second);
    #endif
}

MapPoint::mpptr MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

// void MapPoint::Replace(mpptr pMP, bool bLock)
// {
//     if(pMP->mId==this->mId)
//         return;

//     int nvisible, nfound;
//     map<kfptr,size_t> obs;
//     {
//         unique_lock<mutex> lock1(mMutexFeatures);
//         unique_lock<mutex> lock2(mMutexPos);
//         obs=mObservations;
//         mObservations.clear();
//         mbBad=true;
//         nvisible = mnVisible;
//         nfound = mnFound;
//         mpReplaced = pMP;
//     }

//     for(map<kfptr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
//     {
//         // Replace measurement in keyframe
//         kfptr pKF = mit->first;

//         if(!pMP->IsInKeyFrame(pKF))
//         {
//             pKF->ReplaceMapPointMatch(mit->second, pMP,bLock);
//             pMP->AddObservation(pKF,mit->second,bLock);
//         }
//         else
//         {
//             //MP already has connection to this KF
//             if(mit->second == pMP->GetIndexInKeyFrame(pKF))
//             {
//                 //ids match -- replace old mappoint with new one
//                 pKF->EraseMapPointMatch(mit->second,bLock);
//                 pKF->AddMapPoint(pMP,mit->second,bLock);
//             }
//             else if(pMP->GetIndexInKeyFrame(pKF) >= 0)
//             {
//                 //feat-ids of this MP and replacement MP do not match

//                 //MP knows feat-id in KF, but KF does not know MP -- fix by taking the id with the better descriptor distance
//                 pKF->EraseMapPointMatch(mit->second,bLock);

//                 vector<mpptr> mvpMPs = pKF->GetMapPointMatches();
//                 vector<mpptr>::iterator vit = std::find(mvpMPs.begin(),mvpMPs.end(),pMP);
//                 int id = vit - mvpMPs.begin();

//                 if(id == pMP->GetIndexInKeyFrame(pKF))
//                 {
//                     //pMP thinks it's associated to feat-id x, KF thinks pMP is asscociated to feat-id x -- so just erase connection to THIS (already done)
//                 }
//                 else if(vit == mvpMPs.end())
//                 {
//                     //MP knows KF, but KF does not know MP
//                     //remap to new id or keep old one?

//                     const cv::Mat dMP = pMP->GetDescriptor();

//                     const cv::Mat &dKF_pMP_id = pKF->mDescriptors.row(pMP->GetIndexInKeyFrame(pKF));
//                     const cv::Mat &dKF_this_id = pKF->mDescriptors.row(mit->second);

//                     double dist_pMP_id = ORBmatcher::DescriptorDistance(dMP,dKF_pMP_id);
//                     double dist_this_id = ORBmatcher::DescriptorDistance(dMP,dKF_this_id);

//                     if(dist_pMP_id <= dist_this_id)
//                     {
//                         pKF->AddMapPoint(pMP,pMP->GetIndexInKeyFrame(pKF),false);
//                         //MP already has correct feat-id
//                     }
//                     else
//                     {
//                         pKF->AddMapPoint(pMP,mit->second,bLock);
//                         pMP->EraseObservation(pKF,bLock);
//                         pMP->AddObservation(pKF,mit->second,bLock);
//                     }
//                 }
//                 else
//                 {
//                     //MP thinks it's associated to feat-id x, KF thinks MP is asscociated to feat-id y
//                     cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " id mismatch MP " << pMP->mId.first << endl;
//                 }
//             }
//             else
//             {
//                 cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " id mismatch" << endl;
//                 throw infrastructure_ex();
//             }
//         }
//     }
//     pMP->IncreaseFound(nfound);
//     pMP->IncreaseVisible(nvisible);
//     pMP->ComputeDistinctiveDescriptors();

//     mpMap->EraseMapPoint(this->shared_from_this());
// }

void MapPoint::Replace(mpptr pMP, bool bLock)
{
    if(pMP->mId==this->mId)
        return;

    int nvisible, nfound;
    map<bkfptr,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservationsBKFs;
        mObservationsBKFs.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }
    // 所有能观测到该MapPoint的Bundledkeyframes都要替换
    for(map<bkfptr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
       
        bkfptr pBKFs = mit->first;

        if(!pMP->IsInBundledKeyFrames(pBKFs))
        {
            pBKFs->ReplaceMapPointMatch(mit->second, pMP,bLock);
            pMP->AddBKFsObservation(pBKFs,mit->second, pBKFs->cameraNum, bLock);
        }
        else
        {
            //MP already has connection to this BKF
            if(mit->second == pMP->GetIndexInBundledKeyFrames(pBKFs))
            {
                //ids match -- replace old mappoint with new one
                pBKFs->EraseMapPointMatch(mit->second,bLock);
                pBKFs->AddMapPoint(pMP,mit->second,bLock);
            }
            else if(pMP->GetIndexInBundledKeyFrames(pBKFs) >= 0)
            {
                //feat-ids of this MP and replacement MP do not match

                //MP knows feat-id in KF, but KF does not know MP -- fix by taking the id with the better descriptor distance
                pBKFs->EraseMapPointMatch(mit->second,bLock);

                vector<mpptr> mvpMPs = pBKFs->GetMapPointMatches();
                vector<mpptr>::iterator vit = std::find(mvpMPs.begin(),mvpMPs.end(),pMP);
                int id = vit - mvpMPs.begin();

                if(id == pMP->GetIndexInBundledKeyFrames(pBKFs))
                {
                    //pMP thinks it's associated to feat-id x, KF thinks pMP is asscociated to feat-id x -- so just erase connection to THIS (already done)
                }
                else if(vit == mvpMPs.end())
                {
                    //MP knows KF, but KF does not know MP
                    //remap to new id or keep old one?

                    const cv::Mat dMP = pMP->GetDescriptor();

                    const cv::Mat &dKF_pMP_id = pBKFs->mDescriptors.row(pMP->GetIndexInBundledKeyFrames(pBKFs));
                    const cv::Mat &dKF_this_id = pBKFs->mDescriptors.row(mit->second);

                    double dist_pMP_id = ORBmatcher::DescriptorDistance(dMP,dKF_pMP_id);
                    double dist_this_id = ORBmatcher::DescriptorDistance(dMP,dKF_this_id);

                    if(dist_pMP_id <= dist_this_id)
                    {
                        pBKFs->AddMapPoint(pMP,pMP->GetIndexInBundledKeyFrames(pBKFs),false);
                        //MP already has correct feat-id
                    }
                    else
                    {
                        pBKFs->AddMapPoint(pMP,mit->second,bLock);
                        pMP->EraseBKFsObservation(pBKFs,pBKFs->cameraNum, bLock);
                        pMP->AddBKFsObservation(pBKFs,mit->second, pBKFs->cameraNum, bLock);
                    }
                }
                // else
                // {
                //     //MP thinks it's associated to feat-id x, KF thinks MP is asscociated to feat-id y
                //     cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " id mismatch MP " << pMP->mId.first << endl;
                // }
            }
            else
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " id mismatch" << endl;
                throw infrastructure_ex();
            }
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpBMap->EraseMapPoint(this->shared_from_this());
}

// void MapPoint::ReplaceAndLock(mpptr pMP)
// {
//     if(pMP->mId==this->mId)
//         return;

//     int nvisible, nfound;
//     map<kfptr,size_t> obs;
//     {
//         unique_lock<mutex> lock1(mMutexFeatures);
//         unique_lock<mutex> lock2(mMutexPos);
//         obs=mObservations;
//         mObservations.clear();
//         mbBad=true;
//         nvisible = mnVisible;
//         nfound = mnFound;
//         mpReplaced = pMP;
//     }

//     for(map<kfptr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
//     {
//         // Replace measurement in keyframe
//         kfptr pKF = mit->first;

//         if(pKF->IsEmpty()) continue;

//         if(!pMP->IsInKeyFrame(pKF))
//         {
//             pKF->ReplaceMapPointMatch(mit->second, pMP,true);
//             pMP->AddObservation(pKF,mit->second,true);
//         }
//         else
//         {
//             pKF->EraseMapPointMatch(mit->second,true);
//         }
//     }
//     pMP->IncreaseFound(nfound);
//     pMP->IncreaseVisible(nvisible);
//     pMP->ComputeDistinctiveDescriptors();

//     mpMap->EraseMapPoint(this->shared_from_this());
// }

void MapPoint::ReplaceAndLock(mpptr pMP)
{
    if(pMP->mId==this->mId)
        return;

    int nvisible, nfound;
    map<bkfptr,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservationsBKFs;
        mObservationsBKFs.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<bkfptr,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        bkfptr pBKF = mit->first;

        if(pBKF->IsEmpty()) continue;

        if(!pMP->IsInBundledKeyFrames(pBKF))
        {
            pBKF->ReplaceMapPointMatch(mit->second, pMP,true);
            pMP->AddBKFsObservation(pBKF,mit->second,pBKF->cameraNum,true);
        }
        else
        {
            pBKF->EraseMapPointMatch(mit->second,true);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpBMap->EraseMapPoint(this->shared_from_this());
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(kfptr pKF, bool bIgnoreMutex)
{
    if(!bIgnoreMutex)
        unique_lock<mutex> lock(mMutexFeatures);

    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

int MapPoint::GetIndexInBundledKeyFrames(bkfptr pBKFs, bool bIgnoreMutex)
{
    if(!bIgnoreMutex)
            unique_lock<mutex> lock(mMutexFeatures);

        if(mObservationsBKFs.count(pBKFs))
            return mObservationsBKFs[pBKFs];
        else
            return -1;
}

bool MapPoint::IsInKeyFrame(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

bool MapPoint::IsInBundledKeyFrames(bkfptr pBKFs)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservationsBKFs.count(pBKFs));
}

bool MapPoint::IsObsLocked(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);

    map<kfptr,bool>::iterator mit = mObservationsLock.find(pKF);

    if(mit == mObservationsLock.end())
    {
        return false;
    }
    else
    {
        return mit->second;
    }
}

void MapPoint::UpdateNormalAndDepth()
{
    map<kfptr,size_t> observations;
    kfptr pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<kfptr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        kfptr pKF = mit->first;

        if(pKF->isBad()) continue;

        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}


void MapPoint::UpdateNormalAndDepthPlus()
{   
    map<bkfptr,size_t> observations;
    bkfptr pRefBKFs;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservationsBKFs;
        pRefBKFs=mpRefBKFs;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;

    for(map<bkfptr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        bkfptr pBKFs = mit->first;  //which BKF can observe this mappoint
        if(pBKFs->isBad()) continue;
        vector<int> pair_index = pBKFs->vKeyPointsIndexMapPlus[mit->second];
        if(pair_index[0] >=0)
        {
            cv::Mat Owi = pBKFs->GetCameraCenter(0);
            cv::Mat normali = mWorldPos - Owi;
            normal = normal + normali/cv::norm(normali);
        }
        else if(pair_index[1] >=0)
        {
            cv::Mat Owi = pBKFs->GetCameraCenter(1);
            cv::Mat normali = mWorldPos - Owi;
            normal = normal + normali/cv::norm(normali);
        }
        //todo uncomment
//        else if(pair_index[2] >=0)
//        {
//            cv::Mat Owi = pBKFs->GetCameraCenter(2);
//            cv::Mat normali = mWorldPos - Owi;
//            normal = normal + normali/cv::norm(normali);
//        }
//        else
//        {
//            cv::Mat Owi = pBKFs->GetCameraCenter(3);
//            cv::Mat normali = mWorldPos - Owi;
//            normal = normal + normali/cv::norm(normali);
//        }
        n++;
    }

    size_t index = observations[pRefBKFs];
    int refBKFcameraId = -1;
    vector<int> pair_index = pRefBKFs->vKeyPointsIndexMapPlus[index];

    if(pair_index[0] >=0)
        refBKFcameraId = 0;
    else if(pair_index[1] >=0)
        refBKFcameraId = 1;
//    else if(pair_index[2] >=0)
//        refBKFcameraId = 2;
//    else
//        refBKFcameraId = 3;


    cv::Mat PC = Pos - pRefBKFs->GetCameraCenter(refBKFcameraId);

    const float dist = cv::norm(PC);

    int level;
    if(pRefBKFs->vKeyPointsIndexMapPlus[index][0]>=0)
    {
        int idx = pRefBKFs->vKeyPointsIndexMapPlus[index][0];
        level = pRefBKFs->mvKeysMultipleUn[0][idx].octave;
    }
    else if(pRefBKFs->vKeyPointsIndexMapPlus[index][1]>=0)
    {
        int idx = pRefBKFs->vKeyPointsIndexMapPlus[index][1];
        level = pRefBKFs->mvKeysMultipleUn[1][idx].octave;
    }
    else if(pRefBKFs->vKeyPointsIndexMapPlus[index][2]>=0)
    {
        int idx = pRefBKFs->vKeyPointsIndexMapPlus[index][2];
        level = pRefBKFs->mvKeysMultipleUn[2][idx].octave;
    }
    else
    {
        int idx = pRefBKFs->vKeyPointsIndexMapPlus[index][3];
        level = pRefBKFs->mvKeysMultipleUn[3][idx].octave;
    }


    const float levelScaleFactor =  pRefBKFs->mvScaleFactors[level];
    const int nLevels = pRefBKFs->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/mpRefBKFs->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, kfptr pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, bkfptr pBKFs)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pBKFs->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pBKFs->mnScaleLevels)
        nScale = pBKFs->mnScaleLevels-1;

    return nScale;
}


int MapPoint::PredictScale(const float &currentDist, frameptr pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}


void MapPoint::SendMe()
{
    if(mspComm.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::SendMe(): no Comm ptrs" << endl;
        cout << "bad?: " << (mbBad == true) << endl;
        return;
    }

    if(this->IsSent() && !this->IsInOutBuffer())
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            commptr pComm = *sit;
            pComm->PassMptoComm(this->shared_from_this());
        }
    }
}

void MapPoint::SetSendFull()
{
    {
        unique_lock<mutex> lock(mMutexOut);
        mbSendFull = true;
    }

    this->SendMe();
}

void MapPoint::ReplaceMap(mapptr pNewMap)
{
//    unique_lock<mutex> lockMap(mMapMutex);

    unique_lock<mutex> lockFeat(mMutexFeatures,defer_lock);
    unique_lock<mutex> lockPos(mMutexPos,defer_lock);
    unique_lock<mutex> lockOut(mMutexOut,defer_lock);

    lock(lockFeat,lockPos,lockOut);

    mpMap = pNewMap;
}

void MapPoint::ReplaceBMap(bmapptr pNewBMap)
{
//    unique_lock<mutex> lockMap(mMapMutex);

    unique_lock<mutex> lockFeat(mMutexFeatures,defer_lock);
    unique_lock<mutex> lockPos(mMutexPos,defer_lock);
    unique_lock<mutex> lockOut(mMutexOut,defer_lock);

    lock(lockFeat,lockPos,lockOut);

    mpBMap = pNewBMap;
}

// void MapPoint::ComputeDistinctiveDescriptors()
// {
//     // Retrieve all observed descriptors
//     vector<cv::Mat> vDescriptors;

//     map<kfptr,size_t> observations;

//     {
//         unique_lock<mutex> lock1(mMutexFeatures);
//         if(mbBad)
//             return;
//         observations=mObservations;
//     }

//     if(observations.empty())
//         return;

//     vDescriptors.reserve(observations.size());

//     for(map<kfptr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
//     {
//         kfptr pKF = mit->first;

//         if(!pKF->isBad())
//             vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
//     }

//     if(vDescriptors.empty())
//         return;

//     // Compute distances between them
//     const size_t N = vDescriptors.size();

//     float Distances[N][N];
//     for(size_t i=0;i<N;i++)
//     {
//         Distances[i][i]=0;
//         for(size_t j=i+1;j<N;j++)
//         {
//             int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
//             Distances[i][j]=distij;
//             Distances[j][i]=distij;
//         }
//     }

//     // Take the descriptor with least median distance to the rest
//     int BestMedian = INT_MAX;
//     int BestIdx = 0;
//     for(size_t i=0;i<N;i++)
//     {
//         vector<int> vDists(Distances[i],Distances[i]+N);
//         sort(vDists.begin(),vDists.end());
//         int median = vDists[0.5*(N-1)];

//         if(median<BestMedian)
//         {
//             BestMedian = median;
//             BestIdx = i;
//         }
//     }

//     {
//         unique_lock<mutex> lock(mMutexFeatures);
//         mDescriptor = vDescriptors[BestIdx].clone();
//     }
// }

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<bkfptr,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservationsBKFs;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<bkfptr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        bkfptr pBKFs = mit->first;

        if(!pBKFs->isBad())
            vDescriptors.push_back(pBKFs->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

void MapPoint::ReduceMessage(ccmslam_msgs::MP *pMsgFull, ccmslam_msgs::MPred *pMsgRed)
{
    pMsgRed->mnId = pMsgFull->mnId;
    pMsgRed->mClientId = pMsgFull->mClientId;
    pMsgRed->mUniqueId = pMsgFull->mUniqueId;
    pMsgRed->mPosPred = pMsgFull->mPosPred;
    pMsgRed->mPosPar = pMsgFull->mPosPar;
    pMsgRed->mbNormalAndDepthChanged = pMsgFull->mbNormalAndDepthChanged;
    pMsgRed->mpPredBKFId = pMsgFull->mpPredBKFId;
    pMsgRed->mpPredBKFClientId = pMsgFull->mpPredBKFClientId;
    pMsgRed->mpParBKFId = pMsgFull->mpParBKFId;
    pMsgRed->mpParBKFClientId = pMsgFull->mpParBKFClientId;
    pMsgRed->mbBad = pMsgFull->mbBad;
    pMsgRed->mbMultiUse = pMsgFull->mbMultiUse;
    
}

// void MapPoint::ConvertToMessage(ccmslam_msgs::Map &msgMap, kfptr pRefKf, g2o::Sim3 mg2oS_wcurmap_wclientmap, bool bForceUpdateMsg)
// {
//     unique_lock<mutex> lockOut(mMutexOut);

//     if((mbSendFull || mSysState == eSystemState::SERVER) && !bForceUpdateMsg)
//     {
//         ccmslam_msgs::MP Msg;

//         unique_lock<mutex> lockFeat(mMutexFeatures,defer_lock);
//         unique_lock<mutex> lockPos(mMutexPos,defer_lock);

//         lock(lockFeat,lockPos);

//         Msg.mpPredKFId = static_cast<uint16_t>(pRefKf->mId.first);
//         Msg.mpPredKFClientId = static_cast<uint8_t>(pRefKf->mId.second);

//         if(mSysState == eSystemState::SERVER)
//         {
//             cv::Mat Tpw = pRefKf->GetPose();
//             cv::Mat Rpw = Tpw.rowRange(0,3).colRange(0,3);
//             cv::Mat tpw = Tpw.rowRange(0,3).col(3);

//             cv::Mat P3D_p = Rpw * mWorldPos + tpw;

//             float s = static_cast<double>(mg2oS_wcurmap_wclientmap.inverse().scale());
//             s = 1/s;

//             P3D_p *=(1./s);

//             Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPred_type,float>(P3D_p,Msg.mPosPred);

//             Msg.mpParKFId = KFRANGE;
//             Msg.mpParKFClientId = MAPRANGE;

//             Msg.bSentOnce = mbSentOnce;

//             ccptr pCC = mpMap->GetCCPtr(mId.second);
//             if(pCC->mbOptimized)
//                 Msg.mbPoseChanged = true;
//             else
//                 Msg.mbPoseChanged = mbPoseChanged;

//             Msg.mbServerBA = false;
//         }
//         else
//         {
//             cv::Mat Rcw = pRefKf->GetPose().rowRange(0,3).colRange(0,3);
//             cv::Mat tcw = pRefKf->GetPose().rowRange(0,3).col(3);

//             cv::Mat RefPos = Rcw * mWorldPos + tcw;

//             Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPred_type,float>(RefPos,Msg.mPosPred);

//             if(mpRefKF)
//             {
//                 Msg.mpParKFId = static_cast<uint16_t>(mpRefKF->mId.first);
//                 Msg.mpParKFClientId = static_cast<uint8_t>(mpRefKF->mId.second);

//                 cv::Mat Rparcw = mpRefKF->GetPose().rowRange(0,3).colRange(0,3);
//                 cv::Mat tparcw = mpRefKF->GetPose().rowRange(0,3).col(3);

//                 cv::Mat ParPos = Rparcw * mWorldPos + tparcw;

//                 Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPar_type,float>(ParPos,Msg.mPosPar);
//             }
//             else
//             {
//                 Msg.mpParKFId = KFRANGE;
//                 Msg.mpParKFClientId = MAPRANGE;
//                 cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " no Parent" << endl;
//             }

//             Msg.mbAck = mbAck;

//             Msg.bSentOnce = mbSentOnce;
//             mbSentOnce = true;
//             mbPoseChanged = false;
//             mbSendFull = false;

//             Msg.mbServerBA = false;
//         }

//         //Normal Vector

//         Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mNormalVector_type,float>(mNormalVector,Msg.mNormalVector);

//         //Descriptor

//         Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mDescriptor_type,uint8_t>(mDescriptor,Msg.mDescriptor);

//         //Observation

//         for(std::map<bkfptr,size_t>::const_iterator mit=mObservationsBKFs.begin();mit!=mObservationsBKFs.end();++mit)
//         {
//             if(!mit->first->isBad())
//             {
//                 Msg.mObservations_BKFIDs.push_back(static_cast<uint16_t>(mit->first->mId.first));
//                 Msg.mObservations_BKFClientIDs.push_back(static_cast<uint8_t>(mit->first->mId.second));
//                 Msg.mObservations_n.push_back(static_cast<uint16_t>(mit->second));
//             }
//         }

//         Msg.mfMinDistance = mfMinDistance;
//         Msg.mfMaxDistance = mfMaxDistance;

//         Msg.mbBad = mbBad;
//         Msg.mnId = static_cast<uint32_t>(mId.first);
//         Msg.mClientId = static_cast<uint8_t>(mId.second);
//         Msg.mUniqueId = mUniqueId;

//         Msg.mnFirstBKFid = static_cast<uint16_t>(mFirstBKfsId.first);
//         Msg.mnFirstBKfClientId = static_cast<uint8_t>(mFirstBKfsId.second);

//         Msg.mbMultiUse = mbMultiUse;

//         msgBMap.MapPoints.push_back(Msg);
//     }
//     else
//     {
//         if(mSysState == eSystemState::SERVER)
//         {
//             cout << "\033[1;31m!!!!! FATAL !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " must no be used by server" << endl;
//             throw infrastructure_ex();
//         }

//         ccmslam_msgs::MPred Msg;

//         unique_lock<mutex> lockPos(mMutexPos);

//         if(mbPoseChanged || this->mbMultiUse) // this part should only be called on client. System will send MP without pose change to server to inform it about a "MultiUse-Event"
//         {
//             Msg.mpPredKFId = static_cast<uint16_t>(pRefKf->mId.first);
//             Msg.mpPredKFClientId = static_cast<uint8_t>(pRefKf->mId.second);

//             cv::Mat Rcw = pRefKf->GetPose().rowRange(0,3).colRange(0,3);
//             cv::Mat tcw = pRefKf->GetPose().rowRange(0,3).col(3);

//             cv::Mat RefPos = Rcw * mWorldPos + tcw;

//             Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPred_type,float>(RefPos,Msg.mPosPred);

//             if(mpRefKF)
//             {
//                 Msg.mpParKFId = static_cast<uint16_t>(mpRefKF->mId.first);
//                 Msg.mpParKFClientId = static_cast<uint8_t>(mpRefKF->mId.second);

//                 cv::Mat Rparcw = mpRefKF->GetPose().rowRange(0,3).colRange(0,3);
//                 cv::Mat tparcw = mpRefKF->GetPose().rowRange(0,3).col(3);

//                 cv::Mat ParPos = Rparcw * mWorldPos + tparcw;

//                 Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPar_type,float>(ParPos,Msg.mPosPar);
//             }
//             else
//             {
//                 Msg.mpParKFId = KFRANGE;
//                 Msg.mpParKFClientId = MAPRANGE;
//                 cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " no Parent" << endl;
//             }

//             Msg.mbServerBA = false;

//             mbPoseChanged = false;

//             Msg.mbBad = mbBad;
//             Msg.mnId = static_cast<uint32_t>(mId.first);
//             Msg.mClientId = static_cast<uint8_t>(mId.second);
//             Msg.mUniqueId = mUniqueId;
//             Msg.mbMultiUse = mbMultiUse;
//             Msg.mbAck = mbAck;

//             msgMap.MPUpdates.push_back(Msg);
//         }
//     }
// }

void MapPoint::ConvertToMessage(ccmslam_msgs::BMap &msgBMap, bkfptr pRefBKf, g2o::Sim3 mg2oS_wcurmap_wclientmap, bool bForceUpdateMsg)
{
    unique_lock<mutex> lockOut(mMutexOut);

    if((mbSendFull || mSysState == eSystemState::SERVER) && !bForceUpdateMsg)
    {
        ccmslam_msgs::MP Msg;

        unique_lock<mutex> lockFeat(mMutexFeatures,defer_lock);
        unique_lock<mutex> lockPos(mMutexPos,defer_lock);

        lock(lockFeat,lockPos);

        Msg.mpPredBKFId = static_cast<uint16_t>(pRefBKf->mId.first);
        Msg.mpPredBKFClientId = static_cast<uint8_t>(pRefBKf->mId.second);

        if(mSysState == eSystemState::SERVER)
        {
            cv::Mat Tpw = pRefBKf->GetPose();
            cv::Mat Rpw = Tpw.rowRange(0,3).colRange(0,3);
            cv::Mat tpw = Tpw.rowRange(0,3).col(3);

            cv::Mat P3D_p = Rpw * mWorldPos + tpw;

            float s = static_cast<double>(mg2oS_wcurmap_wclientmap.inverse().scale());
            s = 1/s;

            P3D_p *=(1./s);

            Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPred_type,float>(P3D_p,Msg.mPosPred);

            Msg.mpParBKFId = KFRANGE;
            Msg.mpParBKFClientId = MAPRANGE;

            Msg.bSentOnce = mbSentOnce;

            ccptr pCC = mpBMap->GetCCPtr(mId.second);
            if(pCC->mbOptimized)
                Msg.mbPoseChanged = true;
            else
                Msg.mbPoseChanged = mbPoseChanged;

            Msg.mbServerBA = false;
        }
        else
        {
            cv::Mat Rcw = pRefBKf->GetPose().rowRange(0,3).colRange(0,3);
            cv::Mat tcw = pRefBKf->GetPose().rowRange(0,3).col(3);

            cv::Mat RefPos = Rcw * mWorldPos + tcw;

            Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPred_type,float>(RefPos,Msg.mPosPred);

            if(mpRefBKFs)
            {
                Msg.mpParBKFId = static_cast<uint16_t>(mpRefBKFs->mId.first);
                Msg.mpParBKFClientId = static_cast<uint8_t>(mpRefBKFs->mId.second);

                cv::Mat Rparcw = mpRefBKFs->GetPose().rowRange(0,3).colRange(0,3);
                cv::Mat tparcw = mpRefBKFs->GetPose().rowRange(0,3).col(3);

                cv::Mat ParPos = Rparcw * mWorldPos + tparcw;

                Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPar_type,float>(ParPos,Msg.mPosPar);
            }
            else
            {
                Msg.mpParBKFId = KFRANGE;
                Msg.mpParBKFClientId = MAPRANGE;
                cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " no Parent" << endl;
            }

            Msg.mbAck = mbAck;

            Msg.bSentOnce = mbSentOnce;
            mbSentOnce = true;
            mbPoseChanged = false;
            mbSendFull = false;

            Msg.mbServerBA = false;
        }

        //Normal Vector

        Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mNormalVector_type,float>(mNormalVector,Msg.mNormalVector);

        //Descriptor

        Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mDescriptor_type,uint8_t>(mDescriptor,Msg.mDescriptor);

        //Observation

        for(std::map<bkfptr,size_t>::const_iterator mit=mObservationsBKFs.begin();mit!=mObservationsBKFs.end();++mit)
        {
            if(!mit->first->isBad())
            {
                Msg.mObservations_BKFIDs.push_back(static_cast<uint16_t>(mit->first->mId.first));
                Msg.mObservations_BKFClientIDs.push_back(static_cast<uint8_t>(mit->first->mId.second));
                Msg.mObservations_n.push_back(static_cast<uint16_t>(mit->second));
            }
        }

        Msg.mfMinDistance = mfMinDistance;
        Msg.mfMaxDistance = mfMaxDistance;

        Msg.mbBad = mbBad;
        Msg.mnId = static_cast<uint32_t>(mId.first);
        Msg.mClientId = static_cast<uint8_t>(mId.second);
        Msg.mUniqueId = mUniqueId;

        Msg.mnFirstBKFid = static_cast<uint16_t>(mFirstBKfsId.first);
        Msg.mnFirstBKfClientId = static_cast<uint8_t>(mFirstBKfsId.second);

        Msg.mbMultiUse = mbMultiUse;

        msgBMap.MapPoints.push_back(Msg);
    }
    else
    {
        if(mSysState == eSystemState::SERVER)
        {
            cout << "\033[1;31m!!!!! FATAL !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " must no be used by server" << endl;
            throw infrastructure_ex();
        }

        ccmslam_msgs::MPred Msg;

        unique_lock<mutex> lockPos(mMutexPos);

        if(mbPoseChanged || this->mbMultiUse) // this part should only be called on client. System will send MP without pose change to server to inform it about a "MultiUse-Event"
        {
            Msg.mpPredBKFId = static_cast<uint16_t>(pRefBKf->mId.first);
            Msg.mpPredBKFClientId = static_cast<uint8_t>(pRefBKf->mId.second);

            cv::Mat Rcw = pRefBKf->GetPose().rowRange(0,3).colRange(0,3);
            cv::Mat tcw = pRefBKf->GetPose().rowRange(0,3).col(3);

            cv::Mat RefPos = Rcw * mWorldPos + tcw;

            Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPred_type,float>(RefPos,Msg.mPosPred);

            if(pRefBKf)
            {
                Msg.mpParBKFId = static_cast<uint16_t>(pRefBKf->mId.first);
                Msg.mpParBKFClientId = static_cast<uint8_t>(pRefBKf->mId.second);

                cv::Mat Rparcw = pRefBKf->GetPose().rowRange(0,3).colRange(0,3);
                cv::Mat tparcw = pRefBKf->GetPose().rowRange(0,3).col(3);

                cv::Mat ParPos = Rparcw * mWorldPos + tparcw;

                Converter::CvMatToMsgArrayFixedSize<ccmslam_msgs::MP::_mPosPar_type,float>(ParPos,Msg.mPosPar);
            }
            else
            {
                Msg.mpParBKFId = KFRANGE;
                Msg.mpParBKFClientId = MAPRANGE;
                cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " no Parent" << endl;
            }

            Msg.mbServerBA = false;

            mbPoseChanged = false;

            Msg.mbBad = mbBad;
            Msg.mnId = static_cast<uint32_t>(mId.first);
            Msg.mClientId = static_cast<uint8_t>(mId.second);
            Msg.mUniqueId = mUniqueId;
            Msg.mbMultiUse = mbMultiUse;
            Msg.mbAck = mbAck;

            msgBMap.MPUpdates.push_back(Msg);
        }
    }
}

void MapPoint::UpdateFromMessage(ccmslam_msgs::MP *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    //if MP already exists, we reduce it to a Update-Msg to use common interfaces

    ccmslam_msgs::MPred *pMsgRed = new ccmslam_msgs::MPred();

    this->ReduceMessage(pMsg,pMsgRed);

    this->UpdateFromMessage(pMsgRed,mg2oS_wcurmap_wclientmap);

    delete pMsgRed;
}

void MapPoint::UpdateFromMessage(ccmslam_msgs::MPred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    mbOmitSending = true;

    unique_lock<mutex> lockOut(mMutexOut);
    unique_lock<mutex> lock(mMutexPos);

    if(mSysState == eSystemState::CLIENT)
    {
        bool bSetPos = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

        if(!bSetPos)
        {
            mbOmitSending = false;
            return;
        }

        mbPoseChanged = false;
        mbUpdatedByServer = true;
    }
    else if(mSysState == eSystemState::SERVER)
    {
        if(!mbPoseLock)
        {
            bool bSetPos = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

            if(!bSetPos)
            {
                mbOmitSending = false;
                return;
            }
        }
    }

    if(!mbMultiUse)
        mbMultiUse = pMsg->mbMultiUse;

    mbOmitSending = false;
}

void MapPoint::WriteMembersFromMessage(ccmslam_msgs::MP *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    mbSentOnce=true;

    mId = make_pair(pMsg->mnId,pMsg->mClientId);
    mFirstBKfsId = make_pair(pMsg->mnFirstBKFid,pMsg->mnFirstBKfClientId);

    for(int index=0;index<pMsg->mObservations_BKFIDs.size();++index)
    {
        bkfptr pBKFsi = mpBMap->GetBKfsPtr(pMsg->mObservations_BKFIDs[index],pMsg->mObservations_BKFClientIDs[index]);

        if(pBKFsi && !pBKFsi->isBad())
        {
            mObservationsBKFs[pBKFsi]=pMsg->mObservations_n[index];
            ++nObs;
        }
    }

    if(nObs == 0)
    {
        mbBad = true;
        return;
    }

    mNormalVector = cv::Mat(3,1,5);
    Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mNormalVector_type,float>(mNormalVector,pMsg->mNormalVector);
    mDescriptor = cv::Mat(1,32,0);
    Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mDescriptor_type,uint8_t>(mDescriptor,pMsg->mDescriptor);

    mnFound = nObs;
    mnVisible = nObs;

    mbMultiUse = pMsg->mbMultiUse;

    mWorldPos = cv::Mat(3,1,5);
    mRefPos = cv::Mat(3,1,5);

    if(mSysState == eSystemState::CLIENT)
    {
        bool bSetPos = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

        if(!bSetPos)
        {
            mObservations.clear();
            mbBad = true;
            return;
        }
    }
    else if(mSysState == eSystemState::SERVER)
    {
        bool bSetPos = this->SetPoseFromMessage(pMsg,mg2oS_wcurmap_wclientmap);

        if(!bSetPos)
        {
            mObservations.clear();
            mbBad = true;
            return;
        }
    }
}

bool MapPoint::SetPoseFromMessage(ccmslam_msgs::MP *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    ccmslam_msgs::MPred *pMsgRed = new ccmslam_msgs::MPred();

    this->ReduceMessage(pMsg,pMsgRed);

    bool bReturn = this->SetPoseFromMessage(pMsgRed,mg2oS_wcurmap_wclientmap);

    delete pMsgRed;

    return bReturn;
}

bool MapPoint::SetPoseFromMessage(ccmslam_msgs::MPred *pMsg, g2o::Sim3 mg2oS_wcurmap_wclientmap)
{
    if(mSysState == eSystemState::CLIENT)
    {
        idpair MsgPredId = make_pair(pMsg->mpPredKFId,pMsg->mpPredKFClientId);
        bkfptr pPredBKfs = mpBMap->GetBKfsPtr(MsgPredId); //message sent pose relative to this KF

        if(pPredBKfs && !pPredBKfs->isBad())
        {
            cv::Mat Twp = pPredBKfs->GetPoseInverse();
            cv::Mat Rwp = Twp.rowRange(0,3).colRange(0,3);
            cv::Mat twp = Twp.rowRange(0,3).col(3);

            cv::Mat P3D_p = cv::Mat(3,1,5); //relative to reference
            Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mPosPred_type,float>(P3D_p,pMsg->mPosPred);

            cv::Mat P3D_w = Rwp * P3D_p + twp;

            this->SetWorldPos(P3D_w,false,true);
        }
        else
        {
            //so there is really no pointer available -- we have to ignore this message
            return false;
        }
    }
    else if(mSysState == eSystemState::SERVER)
    {
        cv::Mat P3D_ref = cv::Mat(3,1,5); //in world client

        bkfptr pRef = mpBMap->GetBKfsPtr(pMsg->mpPredBKFId,pMsg->mpPredBKFClientId);

        if(pRef)
        {
            Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mPosPred_type,float>(P3D_ref,pMsg->mPosPred);
        }

        if(!pRef)
        {
            pRef = mpBMap->GetBKfsPtr(pMsg->mpParBKFId,pMsg->mpParBKFClientId);

            if(pRef)
            {
                Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mPosPar_type,float>(P3D_ref,pMsg->mPosPar);
            }
        }

        if(!pRef)
        {
            pRef = mpBMap->GetErasedBKfsPtr(pMsg->mpPredBKFId,pMsg->mpPredBKFClientId);

            if(pRef)
            {
                Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mPosPred_type,float>(P3D_ref,pMsg->mPosPred);
            }
        }

        if(!pRef)
        {
            pRef = mpBMap->GetErasedBKfsPtr(pMsg->mpParBKFId,pMsg->mpParBKFClientId);

            if(pRef)
            {
                Converter::MsgArrayFixedSizeToCvMat<ccmslam_msgs::MP::_mPosPar_type,float>(P3D_ref,pMsg->mPosPar);
            }
        }

        if(!pRef)
        {
            //so there is really no pointer available -- we have to ignore this message
            return false;
        }

        float s = static_cast<double>(mg2oS_wcurmap_wclientmap.scale());
        s = 1/s;

        P3D_ref *=(1./s);

        if(!pRef->isBad())
        {
            cv::Mat Twp =  pRef->GetPoseInverse();

            cv::Mat Rwp = Twp.rowRange(0,3).colRange(0,3);
            cv::Mat twp = Twp.rowRange(0,3).col(3);

            cv::Mat P3D_w = Rwp * P3D_ref + twp;
            P3D_w.copyTo(mWorldPos);
        }
        else
        {
            bkfptr pRefRef = pRef->GetParent();

            if(!pRefRef)
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " parent is bad or not existing" << endl;
                cout << "this: " << this->mId.first << "|" << this->mId.second << endl;
                cout << "pPred: " << pRef->mId.first << "|" << pRef->mId.second << endl;
                cout << "!pPredPred" << endl;
                throw infrastructure_ex();
            }

            cv::Mat T_p_pp = pRef->mTcp;

            while(pRefRef->isBad())
            {
                T_p_pp = T_p_pp * pRefRef->mTcp;
                {
                    cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " parent is bad or not existing" << endl;
                    cout << "this: " << this->mId.first << "|" << this->mId.second << endl;
                    cout << "pPred: " << pRef->mId.first << "|" << pRef->mId.second << endl;
                    cout << "!pPredPred" << endl;
                    throw infrastructure_ex();
                }
            }

            cv::Mat T_pp_w = pRefRef->GetPose();
            cv::Mat Tpw = T_p_pp * T_pp_w;
            cv::Mat Twp = Tpw.inv();

            cv::Mat Rwp = Twp.rowRange(0,3).colRange(0,3);
            cv::Mat twp = Twp.rowRange(0,3).col(3);

            cv::Mat P3D_w = Rwp * P3D_ref + twp;
            P3D_w.copyTo(mWorldPos);
        }
    }

    return true;
}

void MapPoint::EraseInOutBuffer()
{
//    unique_lock<mutex> lockMap(mMutexOut); //is only called by "MapPoint::SetBadFlag()", which already locks this mutex

    if(mspComm.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m MapPoint::EraseInOutBuffer(...): no Comm ptrs" << endl;
        throw infrastructure_ex();
    }

    if(this->IsInOutBuffer())
    {
        for(set<commptr>::const_iterator sit = mspComm.begin();sit!=mspComm.end();++sit)
        {
            commptr pComm = *sit;
            pComm->DeleteMpFromBuffer(shared_from_this());
            mbInOutBuffer = false;
        }
    }
}

bool MapPoint::CanBeForgotten()
{
    unique_lock<mutex> lock(mMutexOut);

    if(mbSentOnce && mbAck && !mbInOutBuffer)
        return true;
    else
        return false;
}

void MapPoint::RemapObservationId(kfptr pKF, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);

    mObservations[pKF] = idx;
}

void MapPoint::RemapObservationId(bkfptr pBKF, const size_t &index)
{
    unique_lock<mutex> lock(mMutexFeatures);

    mObservationsBKFs[pBKF] = index;
}

std::string MapPoint::GetId()
{
    std::stringstream kfid;
    kfid << mId.first << "|" << mId.second;
    return kfid.str();
}

} //end ns
