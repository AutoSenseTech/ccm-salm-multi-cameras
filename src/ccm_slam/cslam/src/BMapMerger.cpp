#include <cslam/BMapMerger.h>

namespace cslam {

BMapMerger::BMapMerger(matchptr pMatcher)
    : bIsBusy(false), mpMatcher(pMatcher)
{
    if(!mpMatcher)
    {
        ROS_ERROR_STREAM("In \" MapMerger::MapMerger()\": nullptr passed");
        throw estd::infrastructure_ex();
    }
}

BMapMerger::bmapptr BMapMerger::MergeBMaps(bmapptr pBMapCurr, bmapptr pBMapMatch, vector<MapMatchHit> vMatchHits)
{
    // Make sure no other module can start GBA
    bool bProblem = false;

    if(pBMapCurr->isNoStartGBA())
    {
        bProblem = true;
    }

    if(pBMapMatch->isNoStartGBA())
    {
        bProblem = true;
    }

    if(bProblem)
    {
        std::cout << __func__ << ":" << __LINE__ << " Waiting for GBA to be able to Start" << std::endl;
        while(pBMapCurr->isNoStartGBA() || pBMapMatch->isNoStartGBA())
        {
            usleep(params::timings::miLockSleep);
        }
        std::cout << __func__ << ":" << __LINE__  << "Continue" << std::endl;
    }

    pBMapCurr->setNoStartGBA();
    pBMapMatch->setNoStartGBA();

    // If a Global Bundle Adjustment is running, abort it
    if(pBMapCurr->isRunningGBA())
        pBMapCurr->StopGBA();

    if(pBMapMatch->isRunningGBA())
        pBMapMatch->StopGBA();

    this->SetBusy();

    bool b0 = false;
    bool b1 = false;
    bool b2 = false;
    bool b3 = false;

    set<ccptr> spCCC = pBMapCurr->GetCCPtrs();
    set<ccptr> spCCM = pBMapMatch->GetCCPtrs();

    #ifdef LOGGING
    ccptr pCClog = *(spCCC.begin());
    pCClog->mpLogger->SetMerge(__LINE__,0);
    #endif

    if(spCCC.size() != pBMapCurr->msuAssClients.size())
    {
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": spCCC.size() != pBMapCurr->msuAssClients.size()" << endl;
        cout << "BMap id: " << pBMapCurr->mBMapId << endl;
        cout << "Associated client IDs:" << endl;
        for(set<size_t>::const_iterator sit = pBMapCurr->msuAssClients.begin();sit!=pBMapCurr->msuAssClients.end();++sit)
            cout << *sit << endl;
        cout << "Associated pCCs:" << endl;
        for(set<ccptr>::const_iterator sit = spCCC.begin();sit!=spCCC.end();++sit)
            cout << (*sit)->mClientId << endl;
        throw estd::infrastructure_ex();
    }

    if(spCCM.size() != pBMapMatch->msuAssClients.size())
    {
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": spCCM.size() != pBMapMatch->msuAssClients.size()" << endl;
        cout << "Map id: " << pBMapMatch->mBMapId << endl;
        cout << "Associated client IDs:" << endl;
        for(set<size_t>::const_iterator sit = pBMapMatch->msuAssClients.begin();sit!=pBMapMatch->msuAssClients.end();++sit)
            cout << *sit << endl;
        cout << "Associated pCCs:" << endl;
        for(set<ccptr>::const_iterator sit = spCCM.begin();sit!=spCCM.end();++sit)
            cout << (*sit)->mClientId << endl;
        throw estd::infrastructure_ex();
    }

    for(set<ccptr>::iterator sit = spCCC.begin();sit!=spCCC.end();++sit)
    {
        ccptr pCC = *sit;

        cout << "spCCC: pCC->mClientId: " << pCC->mClientId << endl;

        if(pCC->mClientId > 3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps()\": associated ClientId out of bounds (" << pCC->mClientId << ")" << endl;
        if(!(pBMapCurr->msuAssClients.count(pCC->mClientId))) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId in pCC but not in msuAssClients" << endl;
        switch(pCC->mClientId)
        {
            case(static_cast<size_t>(0)):
                if(b0) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps()\": associated ClientId found twice" << endl;
                b0 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                #ifdef LOGGING
//                pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                #endif
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(1)):
                if(b1) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps()\": associated ClientId found twice" << endl;
                b1 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                #ifdef LOGGING
//                pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                #endif
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(2)):
                if(b2) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps()\": associated ClientId found twice" << endl;
                b2 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                #ifdef LOGGING
//                pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                #endif
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(3)):
                if(b3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps()\": associated ClientId found twice" << endl;
                b3 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                #ifdef LOGGING
//                pCC->mpLogger->SetMappingLock(__LINE__,pCC->mClientId);
                #endif
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            default: cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps()\": associated ClientId out of bounds" << endl;
        }
    }

    #ifdef LOGGING
    pCClog->mpLogger->SetMerge(__LINE__,0);
    #endif

    for(set<ccptr>::iterator sit = spCCM.begin();sit!=spCCM.end();++sit)
    {
        ccptr pCC = *sit;

        cout << "spCCM: pCC->mClientId: " << pCC->mClientId << endl;

        if(pCC->mClientId > 3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps()\": associated ClientId out of bounds (" << pCC->mClientId << ")" << endl;
        if(!(pBMapMatch->msuAssClients.count(pCC->mClientId))) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps()\": associated ClientId in pCC but not in msuAssClients" << endl;
        switch(pCC->mClientId)
        {
            case(static_cast<size_t>(0)):
                if(b0) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps()\": associated ClientId found twice" << endl;
                b0 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(1)):
                if(b1) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps()\": associated ClientId found twice" << endl;
                b1 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(2)):
                if(b2) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps()\": associated ClientId found twice" << endl;
                b2 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            case(static_cast<size_t>(3)):
                if(b3) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps()\": associated ClientId found twice" << endl;
                b3 = true;
                while(!pCC->LockComm()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockMapping()){usleep(params::timings::miLockSleep);}
                while(!pCC->LockPlaceRec()){usleep(params::timings::miLockSleep);}
                break;
            default: cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps()\": associated ClientId out of bounds" << endl;
        }
    }

    #ifdef LOGGING
    pCClog->mpLogger->SetMerge(__LINE__,0);
    #endif

    // Get Map Mutex
    // Lock all mutexes
    while(!pBMapCurr->LockBMapUpdate()){usleep(params::timings::miLockSleep);}
    while(!pBMapMatch->LockBMapUpdate()){usleep(params::timings::miLockSleep);}

    for(set<ccptr>::iterator sit = spCCC.begin();sit!=spCCC.end();++sit)
    {
        (*sit)->mbOptActive = true;
    }

    for(set<ccptr>::iterator sit = spCCM.begin();sit!=spCCM.end();++sit)
    {
        (*sit)->mbOptActive = true;
    }

    if(pBMapCurr == nullptr || pBMapMatch == nullptr)
    {
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps\": at least one Bmap is nullptr" << endl;
        this->SetIdle();
        return nullptr;
    }

    //create new map
    bmapptr pFusedBMap{new BundledMap(pBMapMatch,pBMapCurr)};
    while(!pFusedBMap->LockBMapUpdate()){usleep(params::timings::miLockSleep);}
    pFusedBMap->UpdateAssociatedData();

    size_t IdC = vMatchHits[0].mpBKFCurr->mId.second;
    size_t IdM = vMatchHits[0].mpBKFMatch->mId.second;

    g2o::Sim3 g2oS_wm_wc; //world match - world curr

    //optimize
    idpair nLoopBKf;

    int idx = 0;
    bkfptr pBKFCur = vMatchHits[idx].mpBKFCurr;
    bkfptr pBKFMatch = vMatchHits[idx].mpBKFMatch;
    g2o::Sim3 g2oScw = vMatchHits[idx].mg2oScw;
    std::vector<mpptr> vpCurrentMatchedPoints = vMatchHits[idx].mvpCurrentMatchedPoints;
    std::vector<mpptr> vpLoopMapPoints = vMatchHits[idx].mvpLoopMapPoints;

    vector<bkfptr> vpBundledKeyFramesCurr = pBMapCurr->GetAllBundledKeyFrames();

    if(IdC != pBKFCur->mId.second || IdM != pBKFMatch->mId.second)
        cout << "\033[1;31m!!! ERROR !!!\033[0m In \"BMapMerger::BMergeMaps\": client ID mismatch" << endl;

    // Ensure current Bundledkeyframe is updated
    pBKFCur->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedBKFs = pBKFCur->GetVectorCovisibleBundledKeyFrames();
    mvpCurrentConnectedBKFs.push_back(pBKFCur);

    nLoopBKf = pBKFCur->mId;

    BundledKeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[pBKFCur]=g2oScw;
    cv::Mat Twc = pBKFCur->GetPoseInverse();

    {
        cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
        cv::Mat twc = Twc.rowRange(0,3).col(3);
        g2o::Sim3 g2oSwc(Converter::toMatrix3d(Rwc),Converter::toVector3d(twc),1.0);
        g2oS_wm_wc = (g2oScw.inverse())*(g2oSwc.inverse()); //todo
    }

    BundledKeyFrameAndPose CorrectedSim3All, NonCorrectedSim3All;
    CorrectedSim3All[pBKFCur]=g2oScw;

    for(vector<bkfptr>::iterator vit=mvpCurrentConnectedBKFs.begin(), vend=mvpCurrentConnectedBKFs.end(); vit!=vend; vit++)
    {
        bkfptr pBKFi = *vit;

        cv::Mat Tiw = pBKFi->GetPose();

        if(pBKFi!=pBKFCur)
        {
            cv::Mat Tic = Tiw*Twc;
            cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
            cv::Mat tic = Tic.rowRange(0,3).col(3);
            g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
            g2o::Sim3 g2oCorrectedSiw = g2oSic*g2oScw;  //todo 
            //Pose corrected with the Sim3 of the loop closure
            CorrectedSim3[pBKFi]=g2oCorrectedSiw;
        }

        cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
        cv::Mat tiw = Tiw.rowRange(0,3).col(3);
        g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
        //Pose without correction
        NonCorrectedSim3[pBKFi]=g2oSiw;
    }
    
    //all bundledkeyframe in the current bmap
    for(vector<bkfptr>::iterator vit = vpBundledKeyFramesCurr.begin();vit!=vpBundledKeyFramesCurr.end();++vit)
    {
        bkfptr pBKFi = *vit;

        BundledKeyFrameAndPose::const_iterator it = CorrectedSim3.find(pBKFi);

        if(it!=CorrectedSim3.end())
        {
            CorrectedSim3All[pBKFi] = it->second;

            BundledKeyFrameAndPose::const_iterator it2 = NonCorrectedSim3.find(pBKFi);
            if(it2==NonCorrectedSim3.end()) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMerger::MergeMaps\": Siw for KF in CorrectedSim3 but not in NonCorrectedSim3" << endl;

            NonCorrectedSim3All[pBKFi] = NonCorrectedSim3[pBKFi];
        }
        else
        {
            cv::Mat Tiw = pBKFi->GetPose();
            //CorrectedSim3All
            cv::Mat Tic = Tiw*Twc;
            cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
            cv::Mat tic = Tic.rowRange(0,3).col(3);
            g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
            g2o::Sim3 g2oCorrectedSiw = g2oSic*g2oScw;
            //Pose corrected with the Sim3 of the loop closure
            CorrectedSim3All[pBKFi]=g2oCorrectedSiw;
            //NonCorrectedSim3All
            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3All[pBKFi]=g2oSiw;
        }
    }

    // Correct MapPoints and BundledKeyFrame of current map
    for(BundledKeyFrameAndPose::iterator mit=CorrectedSim3All.begin(), mend=CorrectedSim3All.end(); mit!=mend; mit++)
    {
        bkfptr pBKFi = mit->first;
        g2o::Sim3 g2oCorrectedSiw = mit->second;
        g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

        g2o::Sim3 g2oSiw =NonCorrectedSim3All[pBKFi];

        vector<mpptr> vpMPsi = pBKFi->GetMapPointMatches();
        for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
        {
            mpptr pMPi = vpMPsi[iMP];
            if(!pMPi)
                continue;
            if(pMPi->isBad())
                continue;
            if(pMPi->mCorrectedByBKF_MM==pBKFCur->mId)
                continue;

            // Project with non-corrected pose and project back with corrected pose
            // 将该未校正的eigP3Dw先从世界坐标系映射到未校正的pKFi相机坐标系，然后再反映射到校正后的世界坐标系下
            cv::Mat P3Dw = pMPi->GetWorldPos();
            Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
            Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

            cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
            pMPi->SetWorldPos(cvCorrectedP3Dw,true);
            pMPi->mCorrectedByBKF_MM = pBKFCur->mId;
            pMPi->mCorrectedReference_MM = pBKFCur->mUniqueId;
            pMPi->UpdateNormalAndDepthPlus();
        }

        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        // 步骤2.3：将Sim3转换为SE3，根据更新的Sim3，更新关键帧的位姿
        Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
        double s = g2oCorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

        pBKFi->SetPose(correctedTiw,true);

        // Make sure connections are updated
        pBKFi->UpdateConnections();

        pBKFi->mCorrected_MM = pBKFCur->mId;
    }

    map<idpair,bkfptr>  mpErasedBKFs = pBMapCurr->GetMmpErasedBundledKeyFrames();
    map<idpair,mpptr>  mpErasedMPs = pBMapCurr->GetMmpErasedMapPoints();

    for(map<idpair,bkfptr>::iterator mitEr = mpErasedBKFs.begin();mitEr != mpErasedBKFs.end();++mitEr)
    {
        bkfptr pBKFi = mitEr->second;

        if(!pBKFi)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": BKF is nullptr" << endl;
            continue;
        }

        if(!pBKFi->isBad())
            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": BKF is erased, but !bad" << endl;

        if(pBKFi->mCorrected_MM == pBKFCur->mId)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": BKF already corrected" << endl;
            continue;
        }

        bkfptr pP = pBKFi->GetParent();
        cv::Mat Tcp = pBKFi->mTcp;
        while(pP->isBad())
        {
            Tcp = pP->mTcp;
            pP = pP->GetParent();
        }

        pBKFi->mCorrected_MM = pBKFCur->mId;
    }

    for(map<idpair,mpptr>::iterator mitEr = mpErasedMPs.begin();mitEr != mpErasedMPs.end();++mitEr)
    {
        mpptr pMPi = mitEr->second;

        if(!pMPi)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": MP is nullptr" << endl;
            continue;
        }

        if(!pMPi->isBad())
            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": MP is erased, but !bad" << endl;

        if(pMPi->mCorrectedByBKF_MM == pBKFCur->mId)
        {
            cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << ": MP already corrected" << endl;
            continue;
        }

        //do not correct erase MPs
    }

    // Start Loop Fusion
    // Update matched map points and replace if duplicated
     // 步骤3：检查当前帧的MapPoints与闭环匹配帧的MapPoints是否存在冲突，对冲突的MapPoints进行替换或填补
    for(size_t i=0; i<vpCurrentMatchedPoints.size(); i++)
    {
        if(vpCurrentMatchedPoints[i])
        {
            mpptr pLoopMP = vpCurrentMatchedPoints[i];
            mpptr pCurMP = pBKFCur->GetMapPoint(i);
            if(pCurMP)
            {
                pCurMP->ReplaceAndLock(pLoopMP);
            }
            else
            {
                pBKFCur->AddMapPoint(pLoopMP,i,true); //lock this MapPoint
                pLoopMP->AddBKFsObservation(pBKFCur,i,pBKFCur->cameraNum,true);
                pLoopMP->ComputeDistinctiveDescriptors();
            }
        }
    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    // 步骤4：通过将闭环时相连关键帧的mvpLoopMapPoints投影到这些关键帧中，进行MapPoints检查与替换
    SearchAndFuse(CorrectedSim3,vpLoopMapPoints);

    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    // 步骤5：更新当前关键帧之间的共视相连关系，得到因闭环时MapPoints融合而新得到的连接关系
    map<bkfptr, set<bkfptr> > LoopConnections;

    for(vector<bkfptr>::iterator vit=mvpCurrentConnectedBKFs.begin(), vend=mvpCurrentConnectedBKFs.end(); vit!=vend; vit++)
    {
        bkfptr pBKFi = *vit;
        vector<bkfptr> vpPreviousNeighbors = pBKFi->GetVectorCovisibleBundledKeyFrames();

        // Update connections. Detect new links.
        pBKFi->UpdateConnections();
        LoopConnections[pBKFi]=pBKFi->GetConnectedBundledKeyFrames();
        for(vector<bkfptr>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pBKFi].erase(*vit_prev);
        }
        //从连接关系中去除闭环之前的一级连接关系，剩下的连接就是由闭环得到的连接关系
        for(vector<bkfptr>::iterator vit2=mvpCurrentConnectedBKFs.begin(), vend2=mvpCurrentConnectedBKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pBKFi].erase(*vit2);
        }
    }

    // Optimize graph
    // 步骤6：进行EssentialGraph优化，LoopConnections是形成闭环后新生成的连接关系，不包括步骤7中当前帧与闭环匹配帧之间的连接关系
    //Optimizer::OptimizeEssentialGraphMapFusion(pFusedBMap, pBKFMatch, pBKFCur, LoopConnections, false);

    // Add loop edge]
     // 步骤7：添加当前帧与闭环匹配帧之间的边（这个连接关系不优化）
    // 这两句话应该放在OptimizeEssentialGraph之前，因为OptimizeEssentialGraph的步骤4.2中有优化
    pBKFMatch->AddLoopEdge(pBKFCur);
    pBKFCur->AddLoopEdge(pBKFMatch);

    cout << "Essential graph optimized" << endl;

    cout << ">>>>> BMapMerger::BMergeMaps --> Global Bundle Adjustment" << endl;

    pFusedBMap->setRunningGBA();
    pFusedBMap->setFinishedGBA();
    pFusedBMap->mbStopGBA = false;

    #ifdef DONOTINTERRUPTMERGE
    pFusedBMap->setMergeStepGBA();
    #endif

    #ifdef DEBUGGING2
    pFusedMap->CheckStructure();
    #endif

    // Launch a new thread to perform Global Bundle Adjustment
    cout << "--- Launch GBA thread" << endl;
    pFusedBMap->mpThreadGBA = new thread(&BMapMerger::RunGBA,this,nLoopBKf,pFusedBMap); //todo

    cout << "\033[1;32;41m!!! MAPS MERGED !!!\033[0m" << endl;
    this->SetIdle();

    //delete old maps and set new ones in threads

    pBMapCurr->SetOutdated();
    pBMapMatch->SetOutdated();

    set<ccptr> spCCF = pFusedBMap->GetCCPtrs();
    for(set<ccptr>::iterator sit = spCCF.begin();sit!=spCCF.end();++sit)
    {
        ccptr pCC = *sit;
        chptr pCH = pCC->mpCH;
        if(spCCC.count(pCC))
        {
            pCH->ChangeBMap(pFusedBMap,g2oS_wm_wc);
            pCC->mbGotMerged = true;
        }
        else
        {
            pCH->ChangeBMap(pFusedBMap,g2o::Sim3());
            //pCH->ClearCovGraph(pBMapCurr->mBMapId); //todo visulization
        }
        pCC->UnLockComm();
        pCC->UnLockPlaceRec();
    }

    pBMapCurr->UnLockBMapUpdate();
    pBMapMatch->UnLockBMapUpdate();
    pFusedBMap->UnLockBMapUpdate();

    pBMapCurr->unsetNoStartGBA();
    pBMapMatch->unsetNoStartGBA();
    pFusedBMap->unsetNoStartGBA();

    #ifdef LOGGING
    pCClog->mpLogger->SetMerge(__LINE__,0);
    #endif

    return pFusedBMap;
}
// 通过将闭环时相连关键帧的MapPoints投影到这些关键帧中，进行MapPoints检查与替换
void BMapMerger::SearchAndFuse(const BundledKeyFrameAndPose &CorrectedPosesBMap, std::vector<mpptr> vpLoopMapPoints)
{
    ORBmatcher matcher(0.8);

    for(BundledKeyFrameAndPose::const_iterator mit=CorrectedPosesBMap.begin(), mend=CorrectedPosesBMap.end(); mit!=mend;mit++)
    {
        bkfptr pBKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<mpptr> vpReplacePoints(vpLoopMapPoints.size(),nullptr);
        matcher.Fuse(pBKF,cvScw,vpLoopMapPoints,4,vpReplacePoints);

        const int nLP = vpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            mpptr pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->ReplaceAndLock(vpLoopMapPoints[i]);
            }
        }
    }
}

void BMapMerger::SetBusy()
{
    unique_lock<mutex> lock(mMutexBusy);
    bIsBusy = true;
}

void BMapMerger::SetIdle()
{
    unique_lock<mutex> lock(mMutexBusy);
    bIsBusy = false;
}

bool BMapMerger::isBusy()
{
    unique_lock<mutex> lock(mMutexBusy);
    return bIsBusy;
}

void BMapMerger::RunGBA(idpair nLoopBKf, bmapptr pFusedBMap)
{
    cout << "-> Starting Global Bundle Adjustment" << endl;

    Optimizer::BMapFusionGBA(pFusedBMap,pFusedBMap->mBMapId,params::opt::mGBAIterations,&(pFusedBMap->mbStopGBA),nLoopBKf,true);

    set<ccptr> spCC = pFusedBMap->GetCCPtrs();

    #ifdef FINALBA
    if(!pFusedMap->mbStopGBA)
    #endif
    {
        unique_lock<mutex> lock(pFusedBMap->mMutexGBA);

        while(!pFusedBMap->LockBMapUpdate()){usleep(params::timings::miLockSleep);}

        cout << "-> Global Bundle Adjustment finished" << endl;
        cout << "-> Updating Bmap ..." << endl;
        int nBadGBA = 0;
        // Correct keyframes starting at map first keyframe
        list<bkfptr> lpBKFtoCheck(pFusedBMap->mvpBundledKeyFramesOrigins.begin(),pFusedBMap->mvpBundledKeyFramesOrigins.end());

        cout << "--> Updating BKFs ..." << endl;

        while(!lpBKFtoCheck.empty())
        {
            bkfptr pBKF = lpBKFtoCheck.front();
            const set<bkfptr> sChilds = pBKF->GetChilds();
            cv::Mat Twc = pBKF->GetPoseInverse();

            //ADD CHECK
            Eigen::Matrix4d testGBA = Converter::toMatrix4d(pBKF->mTcwGBA*pBKF->GetPoseInverse());
            bool bAcceptGBA = false;

            if(testGBA.block<3,1>(0,3).norm()<0.5){
                bAcceptGBA = true;
            }
            else {
                pBKF->mTcwGBA = pBKF->GetPose();
                nBadGBA++;
            }
            //END
            for(set<bkfptr>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
            {
                bkfptr pChild = *sit;
                if(pChild->mBAGlobalForBKFs!=nLoopBKf)
                {
                    cv::Mat Tchildc = pChild->GetPose()*Twc;
                    pChild->mTcwGBA = Tchildc*pBKF->mTcwGBA;
                    #ifdef DEBUGGING2
                    if(!(pChild->mTcwGBA.dims >= 2))
                        std::cout << COUTERROR << " KF" << pChild->mId.first << "|" << pChild->mId.second << ": !(pChild->mTcwGBA.dims >= 2)" << std::endl;
                    #endif
                    pChild->mBAGlobalForBKFs=nLoopBKf;

                }
                lpBKFtoCheck.push_back(pChild);
            }

            #ifdef DEBUGGING2
            if(!(pBKF->mTcwGBA.dims >= 2))
                std::cout << COUTERROR << " BKF" << pBKF->mId.first << "|" << pBKF->mId.second << ": !(pBKF->mTcwGBA.dims >= 2)" << std::endl;
            #endif

            pBKF->mTcwBefGBA = pBKF->GetPose();
            #ifdef DEBUGGING2
            if(!(pBKF->mTcwBefGBA.dims >= 2))
                std::cout << COUTERROR << " BKF" << pBKF->mId.first << "|" << pBKF->mId.second << ": !(pBKF->mTcwBefGBA.dims >= 2)" << std::endl;
            #endif
            if(bAcceptGBA)
                pBKF->SetPose(pBKF->mTcwGBA,true);
            //pKF->SetPose(pKF->mTcwGBA,true);
            lpBKFtoCheck.pop_front();
        }
        cout << "+++++GBA坏掉的帧数(Map Merger)： "<<nBadGBA<<endl;
        cout << "--> Updating MPs ..." << endl;

        // Correct MapPoints
        const vector<mpptr> vpMPs = pFusedBMap->GetAllMapPoints();

        for(size_t i=0; i<vpMPs.size(); i++)
        {
            mpptr pMP = vpMPs[i];

            if(pMP->isBad())
                continue;

            if(pMP->mBAGlobalForBKFs==nLoopBKf)
            {
                // If optimized by Global BA, just update
                #ifdef DEBUGGING2
                if(!(pMP->mPosGBA.dims >= 2))
                    std::cout << COUTERROR << " MP" << pMP->mId.first << "|" << pMP->mId.second << ": !(pMP->mPosGBA.dims >= 2)" << std::endl;
                #endif
                pMP->SetWorldPos(pMP->mPosGBA,true);
            }
            else
            {
                // Update according to the correction of its reference keyframe
                bkfptr pRefBKF = pMP->GetReferenceBundledKeyFrame();

                if(!pRefBKF)
                {
                    cout << "\033[1;31m!!! ERROR !!!\033[0m In \"LoopFinder::CorrectLoop()\": pRefBKF is nullptr" << endl;
                    continue;
                }

                if(pRefBKF->mBAGlobalForBKFs!=nLoopBKf)
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
            }
        }

        cout << "-> Map updated!" << endl;

        #ifdef FINALBA
        pFusedBMap->unsetGBAinterrupted();
        #endif

        #ifdef DONOTINTERRUPTMERGE
        pFusedBMap->unsetMergeStepGBA();
        #endif

        pFusedBMap->UnLockBMapUpdate();
    }
    #ifdef FINALBA
    else
    {
        cout << COUTNOTICE << "GBA interrupted" << endl;

        #ifdef DONOTINTERRUPTMERGE
        if(pFusedBMap->isMergeStepGBA())
        {
            cout << COUTFATAL << endl;
            KILLSYS
        }
        #endif

        pFusedBMap->setGBAinterrupted();
    }
    #endif
    //todo  write file
    // if(params::stats::mbWriteKFsToFile)
    // {
    //     for(int it=0;it<4;++it)
    //     {
    //         std::stringstream ss;
    //         ss << params::stats::msOutputDir << "BKF_GBA_" << it << ".csv";
    //         pFusedBMap->WriteStateToCsv(ss.str(),it);
    //     }
    // }

    pFusedBMap->setFinishedGBA();
    pFusedBMap->unsetRunningGBA();

    for(set<ccptr>::iterator sit = spCC.begin();sit!=spCC.end();++sit)
    {
        ccptr pCC = *sit;
        pCC->mbOptimized= true;

        pCC->UnLockMapping();

        pCC->mbOptActive = false;
    }

    cout << "-> Leave Thread" << endl;
}

}
