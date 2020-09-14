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

#include <cslam/MapMatcher.h>

namespace cslam {

MapMatcher::MapMatcher(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, dbptr pDB, vocptr pVoc, mapptr pMap0, mapptr pMap1, mapptr pMap2, mapptr pMap3)
    : mNh(Nh), mNhPrivate(NhPrivate),
      mpKFDB(pDB), mpVoc(pVoc), mpMap0(pMap0), mpMap1(pMap1), mpMap2(pMap2), mpMap3(pMap3),
      mLastLoopKFid(0),
      mbFixScale(false),
      mnCovisibilityConsistencyTh(params::placerec::miCovisibilityConsistencyTh)
{

    if(pMap0) mmpMaps[*(pMap0->msuAssClients.begin())]=pMap0;
    if(pMap1) mmpMaps[*(pMap1->msuAssClients.begin())]=pMap1;
    if(pMap2) mmpMaps[*(pMap2->msuAssClients.begin())]=pMap2;
    if(pMap3) mmpMaps[*(pMap3->msuAssClients.begin())]=pMap3;

    if(pMap0) mspMaps.insert(pMap0);
    if(pMap1) mspMaps.insert(pMap1);
    if(pMap2) mspMaps.insert(pMap2);
    if(pMap3) mspMaps.insert(pMap3);

    mPubMarker = mNh.advertise<visualization_msgs::Marker>("MapMatcherMarkers",10);

    mMatchMatrix =  cv::Mat::zeros(4,4,2);

    mMapMatchEdgeMsg.header.frame_id = "world";
    mMapMatchEdgeMsg.header.stamp = ros::Time::now();
    mMapMatchEdgeMsg.ns = "MapMatchEdges_red";
    mMapMatchEdgeMsg.type = visualization_msgs::Marker::LINE_LIST;
    mMapMatchEdgeMsg.color = Colors::msgRed();
    mMapMatchEdgeMsg.action = visualization_msgs::Marker::ADD;
    mMapMatchEdgeMsg.scale.x = params::vis::mfLoopMarkerSize;
    mMapMatchEdgeMsg.id = 1;
}


MapMatcher::MapMatcher(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, bdbptr pBDB, vocptr pVoc, bmapptr pBMap0, bmapptr pBMap1, bmapptr pBMap2, bmapptr pBMap3)
    : mNh(Nh), mNhPrivate(NhPrivate),
      mpBKFDB(pBDB), mpVoc(pVoc), mpBMap0(pBMap0), mpBMap1(pBMap1), mpBMap2(pBMap2), mpBMap3(pBMap3),
      mLastLoopBKFid(0),
      mbFixScale(false),
      mnCovisibilityConsistencyTh(params::placerec::miCovisibilityConsistencyTh)
{
    if(pBMap0) mmpBMaps[*(pBMap0->msuAssClients.begin())]=pBMap0;
    if(pBMap1) mmpBMaps[*(pBMap1->msuAssClients.begin())]=pBMap1;
    if(pBMap2) mmpBMaps[*(pBMap2->msuAssClients.begin())]=pBMap2;
    if(pBMap3) mmpBMaps[*(pBMap3->msuAssClients.begin())]=pBMap3;

    if(pBMap0) mspBMaps.insert(pBMap0);
    if(pBMap1) mspBMaps.insert(pBMap1);
    if(pBMap2) mspBMaps.insert(pBMap2);
    if(pBMap3) mspBMaps.insert(pBMap3);

    mPubMarker = mNh.advertise<visualization_msgs::Marker>("MapMatcherMarkers",10);

    mMatchMatrix =  cv::Mat::zeros(4,4,2);

    mMapMatchEdgeMsg.header.frame_id = "world";
    mMapMatchEdgeMsg.header.stamp = ros::Time::now();
    mMapMatchEdgeMsg.ns = "MapMatchEdges_red";
    mMapMatchEdgeMsg.type = visualization_msgs::Marker::LINE_LIST;
    mMapMatchEdgeMsg.color = Colors::msgRed();
    mMapMatchEdgeMsg.action = visualization_msgs::Marker::ADD;
    mMapMatchEdgeMsg.scale.x = params::vis::mfLoopMarkerSize;
    mMapMatchEdgeMsg.id = 1;
}

void MapMatcher::Run()
{
    double CovGraphMarkerSize;
    mNhPrivate.param("MarkerSizeServer",CovGraphMarkerSize,0.001);
    mpBMapMerger.reset(new BMapMerger(shared_from_this()));

    #ifdef LOGGING
    KeyFrame::ccptr pCC = mpMap0->GetCCPtr(0);
    if(!pCC)
    {
        std::cout << COUTERROR << "pCC not valid" << std::endl;
        KILLSYS
    }
    #endif

    while(1)
    {
        #ifdef LOGGING
        pCC->mpLogger->SetMatch(__LINE__,0);
        #endif
		bool isSim3 =false;
        if(mScw.empty())
        {
            if(CheckBKfQueue())
            {
                cout<<"Enter 线程 4"<<endl;
                bool bDetect = DetectLoop();
                cout<<"bDetect: "<<bDetect<<endl;
                if(bDetect)
                {
                    cout<<"compute sim3"<<endl;
                    bool bSim3 = ComputeSim3();
                    cout<<"compute sim3 finish"<<endl;
                    isSim3 = bSim3;
                    if(bSim3)
                    {
                        //cout<<"you sim3"<<endl;
                        // Perform loop fusion and pose graph optimization
                        CorrectLoop();
                        cout<<"CorrectLoop finish"<<endl;
                    }
                }
            }
        }
        // cv::Mat tmp = cv::Mat::ones(4,4, CV_32F);
        // tmp.copyTo(mScw);
        //TODO
        // if(!mScw.empty())//暂时先不运行
        // {
        //     // cv::Mat tmp 
        //     //cout<<"++++++++++++++++++++++++++++++++++这里是mapmatch++++++++++++++++"<<endl;
        //     mmSkelepos=mpComm->GetSkelepos();
        //     if(!mmSkelepos.empty()&&!mmSkelepos.rbegin()->second.empty())
        //     {
        //         //cout<<"+++++server time & pos size: "<<mmSkelepos.begin()->first<<", "<<mmSkelepos.begin()->second.size()<<endl;
        //         //cout<<"++++++++++++++++++++++++++++++++++not empty1++++++++++++++++"<<endl;
        //         double timeThreshold=0.5;
        //         kfptr cloestKF = FindClosestKeyFrame(timeThreshold);
        //         //cout<<"++++++++++++++++++++++++++++++++++not empty2++++++++++++++++"<<endl;
        //         if(cloestKF)
        //         {
        //             vector<cv::Mat> clientoPos = TransferToCurrentFrame(mmSkelepos, cloestKF);
        //             //cout<<"++++++++++++++++++++++++++++++++++not empty3++++++++++++++++"<<endl;
        //             map<double, vector<float>> result =  TransferToOtherFrame(clientoPos);
        //             //cout<<"+++++++++++++update skele posinputPose!+++++++++++++++"<<result.begin()->second.size()<<endl;
        //             mpComm1->PassSkePointstoComm(result);
        //             //cout<<"++++++++++++++++++++++++++++++++++not empty4++++++++++++++++"<<endl;
        //         }

        //     }
            
        // }
        //ENDTO

        #ifdef LOGGING
        pCC->mpLogger->SetMatch(__LINE__,0);
        #endif

        usleep(params::timings::server::miPlaceRecRateRate);
    }
}

MapMatcher::kfptr MapMatcher::FindClosestKeyFrame(const double &timeThreshold)
{
    //add mutex here
    double skeleposTimeStamp = mmSkelepos.rbegin()->first;
    //cout<<"+++++++current frame clinet id: "<<mpCurrentKF->mId.second<<endl;
    //cout<<"skeleton time: "<<setw(20)<<setprecision(15)<<setiosflags(ios::fixed)<<skeleposTimeStamp<<endl;
    //cout<<"current time: "<<setw(20)<<setprecision(15)<<setiosflags(ios::fixed)<<mpCurrentKF->mTimeStamp<<endl;
    mpCurrMap = mpCurrentKF->GetMapptr();//@@wwh
    vector<kfptr> allKeyFrames = mpCurrMap->GetAllKeyFrames();
    if(allKeyFrames.empty())
        cout<<"error empty"<<endl;
    double minTimeStamp = INT_MAX * 1.0;
    kfptr clostKF = nullptr;
    for(kfptr kf:allKeyFrames)
    {
        if(kf->mId.second == 0)
        {
            double interval = kf->mTimeStamp - skeleposTimeStamp;
            interval = interval > 0 ? interval : -1*interval;
            cout<<"interval: "<<interval<<endl;
            if(interval < minTimeStamp)
            {
                minTimeStamp = interval;
                clostKF = kf;
            }
        }         
    }

    if(minTimeStamp <= timeThreshold)
        return clostKF;
    else
        cout<<"greater than threshold: "<<minTimeStamp<<endl;
    return nullptr;   
}
//client 0 frame
vector<cv::Mat> MapMatcher::TransferToCurrentFrame(map<double, vector<float>> inputPos, kfptr pclient0kf)
{
    vector<cv::Mat> res;
    vector<float> skeleposInof = inputPos.rbegin()->second;
    //cout<<"1"<<endl;
    Eigen::Matrix<double,3,3> K = Converter::toMatrix3d(mpCurrentKF->mK);
    //cout<<"2"<<endl;
    float u = 0.0, v = 0.0, depth = 0.0;
    if(pclient0kf)
    {
        cv::Mat R = pclient0kf->GetPoseInverse().rowRange(0,3).colRange(0,3);  //Rwc
        //cout<<"3"<<endl;
        cv::Mat t = pclient0kf->GetPoseInverse().rowRange(0,3).col(3); //twc
        //cout<<"4"<<endl;
        for(int i = 0; i < skeleposInof.size();i++)
        {

            if(i%3 == 0)
            {
                u = skeleposInof[i];
            }
            else if(i%3==1)
            {
                v = skeleposInof[i];
            }else
            {
                depth = skeleposInof[i];
                Eigen::Vector3d pix, position;
                pix << depth*u, depth*v, depth;
                position = K.inverse() * pix;
                //cout<<"position: "<<position.transpose()<<endl;
                res.push_back(R*Converter::toCvMat(position)+t);
            }
        }
        return res;
    }
}

//step 4  clint 1 world
map<double, vector<float>> MapMatcher::TransferToOtherFrame(const vector<cv::Mat> &client0Pos)
{
    map<double, vector<float>> result;
    vector<float> res;
    //cout<<"TransferToOtherFrame client0Pos size:"<<client0Pos.size()<<endl;
    if(mpCurrentKF->mId.second == 0 && mpMatchedKF->mId.second == 1)
    {
        cout<<"------------chose client 0"<<endl;
        cv::Mat sRcw = mScw.rowRange(0,3).colRange(0,3);  //0->1
        const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));   // compute scale s
        cv::Mat Rcw = sRcw/scw;
        cv::Mat Rwc = Rcw.t();
        cv::Mat tcw = mScw.rowRange(0,3).col(3)/scw; 
        cv::Mat twc = -Rcw.t()*tcw;
        for(int i = 0; i < client0Pos.size(); i++)
        {
            cv::Mat pos = Rwc*client0Pos[i]+twc; 
            res.push_back(pos.at<float>(0));
            res.push_back(pos.at<float>(1));
            res.push_back(pos.at<float>(2));
            // res.push_back(pos); 
            //cout<<"after position: "<<pos.t()<<endl;
        }
    }else if(mpCurrentKF->mId.second == 1 && mpMatchedKF->mId.second == 0)
    {
        cout<<"------------chose client 1"<<endl;
        cv::Mat sRcw = mScw.rowRange(0,3).colRange(0,3);  //1->0 client1 to client 0 world
        const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));   // compute scale s
        cv::Mat Rcw = sRcw/scw;
        cv::Mat Rwc = Rcw.t();
        cv::Mat tcw = mScw.rowRange(0,3).col(3)/scw; 
        cv::Mat twc = -Rcw.t()*tcw;

        cv::Mat R = mpCurrentKF->GetPoseInverse().rowRange(0,3).colRange(0,3);  //client1 to client1 world  Twc
        cv::Mat t = mpCurrentKF->GetPoseInverse().rowRange(0,3).col(3);

        for(int i = 0; i < client0Pos.size(); i++)
        {
            cv::Mat pos = R*(Rcw*client0Pos[i]+tcw)+t; 
           
            res.push_back(pos.at<float>(0));
            res.push_back(pos.at<float>(1));
            res.push_back(pos.at<float>(2));
            // res.push_back(pos);
        }
    }
    double skeleposTimeStamp = mmSkelepos.rbegin()->first;
    //cout<<"+++++++++++++res size:"<<res.size()<<endl;
    result[skeleposTimeStamp] = res;
    //cout<<"time: "<<result.begin()->first<<endl;
    //cout<<"+++++++++++++result size:"<<result.begin()->second.size()<<endl;
    return result;
}
bool MapMatcher::DetectLoop()
{
    {
        unique_lock<mutex> lock(mMutexBKfInQueue);
        mpCurrentBKF = mlBKfsInQueue.front();

        mlBKfsInQueue.pop_front();
        // Avoid that a Bundledkeyframe can be erased while it is being process by this thread
        mpCurrentBKF->SetNotErase();
    }

    //If the Bmap contains less than 10 BKF or less than 10 BKF have passed from last loop detection
    // 步骤1：如果距离上次闭环没多久（小于10帧），或者map中关键帧总共还没有30帧，则不进行闭环检测
    if(mpCurrentBKF->mId.first<params::placerec::miStartMapMatchingAfterKf) 
    {
        mpCurrentBKF->SetErase();
        return false;
    }

    mpCurrBMap = mpCurrentBKF->GetBMapptr(); //get map of KF

    if(!mpCurrBMap)
    {
        cout << ": In \"MapMatcher::DetectLoop()\": mpCurrBMap is nullptr -> KF not contained in any map" << endl;
        throw estd::infrastructure_ex();
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    //步骤2：遍历所有共视关键帧，计算当前关键帧与每个共视关键的bow相似度得分，并得到最低得分minScore
    const vector<bkfptr> vpConnectedBundledKeyFrames = mpCurrentBKF->GetVectorCovisibleBundledKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentBKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedBundledKeyFrames.size(); i++)
    {
        bkfptr pBKF = vpConnectedBundledKeyFrames[i];
        if(pBKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pBKF->mBowVec;

        float score = mpVoc->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    vector<bkfptr> vpCandidateBKFs = mpBKFDB->DetectMapMatchCandidates(mpCurrentBKF, minScore, mpCurrBMap);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateBKFs.empty())
    {
        mmvConsistentGroups[mpCurrBMap].clear();
        mpCurrentBKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    // 步骤4：在候选帧中检测具有连续性的候选帧
    // 1、每个候选帧将与自己相连的关键帧构成一个“子候选组spCandidateGroup”，vpCandidateKFs-->spCandidateGroup
    // 2、检测“子候选组”中每一个关键帧是否存在于“连续组”，如果存在nCurrentConsistency++，则将该“子候选组”放入“当前连续组vCurrentConsistentGroups”
    // 3、如果nCurrentConsistency大于等于3，那么该”子候选组“代表的候选帧过关，进入mvpEnoughConsistentCandidates
    mvpEnoughConsistentCandidates.clear();

    // ConsistentGroup.first对应每个“连续组”中的关键帧，ConsistentGroup.second为每个“连续组”的序号
    vector<ConsistentGroup> vCurrentConsistentGroups; // pair<set<bkfptr>,int> ConsistentGroup; --> int counts consistent groups found for this group 
    vector<bool> vbConsistentGroup(mmvConsistentGroups[mpCurrBMap].size(),false);
    //mvConsistentGroups stores the last found consistent groups.

    for(size_t i=0, iend=vpCandidateBKFs.size(); i<iend; i++)
    {
        bkfptr pCandidateBKF = vpCandidateBKFs[i];

        set<bkfptr> spCandidateGroup = pCandidateBKF->GetConnectedBundledKeyFrames();
        spCandidateGroup.insert(pCandidateBKF);
        //group with candidate and connected KFs

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        // 遍历之前的“子连续组”
        for(size_t iG=0, iendG=mmvConsistentGroups[mpCurrBMap].size(); iG<iendG; iG++)
        {
            set<bkfptr> sPreviousGroup = mmvConsistentGroups[mpCurrBMap][iG].first;

            bool bConsistent = false;
            for(set<bkfptr>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    //BKF found that is contained in candidate's group and comparison group
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mmvConsistentGroups[mpCurrBMap][iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    // 将该“子候选组”的该关键帧打上编号加入到“当前连续组”
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateBKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0); //For "ConsistentGroup" the "int" is initialized with 0
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mmvConsistentGroups[mpCurrBMap] = vCurrentConsistentGroups;

    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentBKF->SetErase();
        return false;
    }
    else
    {
        return true;
    }

    mpCurrentBKF->SetErase();
    return false;
}

bool MapMatcher::ComputeSim3()
{
    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<mpptr> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    for(int i=0; i<nInitialCandidates; i++)
    {
        bkfptr pBKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pBKF->SetNotErase();

        if(pBKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentBKF,pBKF,vvpMapPointMatches[i]);

        if(nmatches<params::opt::mMatchesThres)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            // ADD 双目server
            mbFixScale = true;
            // 构造Sim3求解器
            // 如果mbFixScale为true，则是6DoFf优化（双目 RGBD），如果是false，则是7DoF优化（单目）
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentBKF,pBKF,vvpMapPointMatches[i],mbFixScale);
            pSolver->SetRansacParameters(params::opt::mProbability,params::opt::mMinInliers,params::opt::mMaxIterations);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    bool bMatch = false;
    //cout<<"++ candiates个数： "<<nCandidates<<endl;
    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    // 一直循环所有的候选帧，每个候选帧迭代5次，如果5次迭代后得不到结果，就换下一个候选帧
    // 直到有一个候选帧首次迭代成功bMatch为true，或者某个候选帧总的迭代次数超过限制，直接将它剔除
    while(nCandidates>0 && !bMatch)
    {
        //cout<<"++++++++进入计算sim3 有candiates"<<endl;
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            bkfptr pBKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(params::opt::mSolverIterations,bNoMore,vbInliers,nInliers);
            //之前的复杂想法
            // pair<int,int> bestcamId1camId2 = pSolver->bestcamId1camId2;
            // cv::Mat T0i_1 = mpCurrentBKF->vmTi0[bestcamId1camId2.first].inv();
            // cv::Mat R0i_1 = T0i_1.rowRange(0,3).colRange(0,3);
            // cv::Mat t0i_1 = T0i_1.rowRange(0,3).col(3);

            // cv::Mat Ti0_2 = pBKF->vmTi0[bestcamId1camId2.second];
            // cv::Mat Ri0_2 = Ti0_2.rowRange(0,3).colRange(0,3);
            // cv::Mat ti0_2 = Ti0_2.rowRange(0,3).col(3);

            // Scm = T0i_1*Scm*Ti0_2;
            //end 复杂想法

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<mpptr> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<mpptr>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                //R = R0i_1 * R * Ri0_2;  //R = (cameraBKF,CandiateBKF)
                cv::Mat t = pSolver->GetEstimatedTranslation();
                //t = R0i_1*R*ti0_2 + R0i_1*t + t0i_1;

                const float s = pSolver->GetEstimatedScale();
                matcher.SearchBySim3(mpCurrentBKF,pBKF,vpMapPointMatches,s,R,t,7.5); //制造更多的匹配的mappoint

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentBKF, pBKF, vpMapPointMatches, gScm, 10, mbFixScale);
                // If optimization is succesful stop ransacs and continue
                if(nInliers>=params::opt::mInliersThres)
                {
                    //cout<<"Enter Here Inliers"<<endl;
                    bMatch = true;
                    mpMatchedBKF = pBKF;
                    //cout<<"Enter Here Inliers 1"<<endl;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pBKF->GetRotation()),Converter::toVector3d(pBKF->GetTranslation()),1.0);
                   // 得到g2o优化后从世界坐标系到当前帧的Sim3变换
                    mg2oScw = gScm*gSmw;
                    //cout<<"Enter Here Inliers 3"<<endl;
                    mScw = Converter::toCvMat(mg2oScw);
                //     size_t clientIDCur = mpCurrentKF->mId.second;
                //    // cout<<"Enter Here Inliers 4" <<endl;
                //     //cout<<"client id: "<<clientIDCur<<endl;
                //     size_t clientIDMat = mpMatchedKF->mId.second;
                //     //cout<<"Enter Here Inliers 5"<<endl;
                //     //cout<<"client id(match): "<<clientIDMat<<endl;
                //     kfptr curIntKf = mpMap0->GetKfPtr(0,0);
                //     // if(curIntKf){
                //     //     cout<<"Id current:"<<curIntKf->mId.first<<endl;
                //     // }
                //    // cout<<"Enter Here Inliers 6"<<endl;
                //     kfptr matIntKf = mpMap1->GetKfPtr(0,1);  
                //     // if(curIntKf){
                //     //     cout<<"Id current:"<<matIntKf->mId.first<<endl;
                //     // }               
                //     //cout<<"current Map 第一帧时间戳： "<<setprecision(25)<<curIntKf->mTimeStamp<<endl;
                //     //cout<<"match Map 第一帧时间戳： "<<setprecision(25)<<matIntKf->mTimeStamp<<endl;
                //     if(curIntKf->mTimeStamp - matIntKf->mTimeStamp < 1e-6)
                //     {
                //         cout<<"ENTER HERE: SAME FRAME"<<endl;
                //         Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
                //         Eigen::Vector3d t0 = Eigen::Vector3d::Zero();
                //         Eigen::Matrix4d estim_SE3 = Converter::toMatrix4d(mpCurrentKF->GetPose()*mpMatchedKF->GetPoseInverse());
                //         Eigen::Matrix3d R_estim = estim_SE3.block<3,3>(0,0);
                //         Eigen::Vector3d t_estim = estim_SE3.block<3,1>(0,3);
                //         // g2o::Sim3 temp_gScm(R_estim, t_estim,1.0);
                //         // mg2oScw = temp_gScm*gSmw;
                //         // mScw = Converter::toCvMat(mg2oScw);
                //         // cout<< "+++++++++++++++ccm compute sim3: " << endl << Converter::toCvMat(gScm)<<endl;
                //         // cout<< "+++++++++++++++ccm compute sim3/s: " << s << endl;
                //         // cout<< "++++++++++++++estimate compute sim3: "<<endl <<estim_SE3<<endl;
                //         //cout<<"current Map ClientID： "<<clientIDCur<<endl;
                //         //cout<<"match Map ClientID： "<<clientIDMat<<endl;
                //     }
                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        //cout<<"进入计算sim3 !bmatch"<<endl;
        for(int i=0; i<nInitialCandidates; i++)
             mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentBKF->SetErase();
        return false;
    }

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<bkfptr> vpLoopConnectedBKFs = mpMatchedBKF->GetVectorCovisibleBundledKeyFrames();
    vpLoopConnectedBKFs.push_back(mpMatchedBKF);
    mvpLoopMapPoints.clear();
    for(vector<bkfptr>::iterator vit=vpLoopConnectedBKFs.begin(); vit!=vpLoopConnectedBKFs.end(); vit++)
    {
        bkfptr pBKF = *vit;
        vector<mpptr> vpMapPoints = pBKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            mpptr pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mLoopPointForBKF_MM!=mpCurrentBKF->mId) //ID Tag
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mLoopPointForBKF_MM = mpCurrentBKF->mId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentBKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches>=params::opt::mTotalMatchesThres)
    {
        for(int i=0; i<nInitialCandidates; i++)
            if(mvpEnoughConsistentCandidates[i]!=mpMatchedBKF)
                mvpEnoughConsistentCandidates[i]->SetErase();
        return true;
    }
    else
    {
        //cout<<"进入计算sim3 params::opt::mTotalMatchesThres"<<endl;
        for(int i=0; i<nInitialCandidates; i++)
            mvpEnoughConsistentCandidates[i]->SetErase();
        mpCurrentBKF->SetErase();
        return false;
    }
}

void MapMatcher::CorrectLoop()
{
    cout << "\033[1;32m!!! MAP MATCH FOUND !!!\033[0m" << endl;

    set<size_t> suAssCLientsCurr = mpCurrentBKF->GetBMapptr()->msuAssClients;
    set<size_t> suAssCLientsMatch = mpMatchedBKF->GetBMapptr()->msuAssClients;

    for(set<size_t>::iterator sit = suAssCLientsCurr.begin();sit!=suAssCLientsCurr.end();++sit)
    {
        size_t idc = *sit;
        for(set<size_t>::iterator sit2 = suAssCLientsMatch.begin();sit2!=suAssCLientsMatch.end();++sit2)
        {
            size_t idm = *sit2;

            if(idc == idm) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Associated Clients of matched and current map intersect" << endl;

            mMatchMatrix.at<uint16_t>(idc,idm) = mMatchMatrix.at<uint16_t>(idc,idm) + 1;
            mMatchMatrix.at<uint16_t>(idm,idc) = mMatchMatrix.at<uint16_t>(idm,idc)  + 1;
        }
    }

    if(mpCurrentBKF->mId.second == mpMatchedBKF->mId.second) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Matched KFs belong to same client" << endl;
    if(!mpCurrBMap->msuAssClients.count(mpCurrentBKF->mId.second)) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Current KFs does not belong to current map" << endl;
    if(mpCurrBMap->msuAssClients.count(mpMatchedBKF->mId.second)) cout << "\033[1;31m!!! ERROR !!!\033[0m In \"MapMatcher::CorrectLoop()\": Matched KFs belongs to current map" << endl;

    //todo for visualization 
    // if(params::vis::mbActive)
    //     PublishLoopEdges();
    //end todo

    bmapptr pMatchedBMap = mpMatchedBKF->GetBMapptr();

    MapMatchHit MMH(mpCurrentBKF,mpMatchedBKF,mg2oScw,mvpLoopMapPoints,mvpCurrentMatchedPoints);
    mFoundMatches[mpCurrBMap][pMatchedBMap].push_back(MMH);
    mFoundMatches[pMatchedBMap][mpCurrBMap].push_back(MMH);

    if(mFoundMatches[mpCurrBMap][pMatchedBMap].size() >= 1)
    {
        vector<MapMatchHit> vMatches = mFoundMatches[mpCurrBMap][pMatchedBMap];
        //cout<<"++++++++开始MergeMap，新地图时间： "<<setprecision(25)<<mpCurrentKF->mTimeStamp<<endl;
        bmapptr pMergedBMap = mpBMapMerger->MergeBMaps(mpCurrBMap,pMatchedBMap,vMatches);
    }

    //this->ClearLoopEdges(); //todo uncomment
}

void MapMatcher::PublishLoopEdges()
{
    mMapMatchEdgeMsg.points.clear();

    tf::StampedTransform Tf_W_Curr,Tf_W_Matched;
    string FrameIdCurr,FrameIdMatched;

    FrameIdCurr = mpCurrentKF->GetMapptr()->mOdomFrame;
    FrameIdMatched = mpMatchedKF->GetMapptr()->mOdomFrame;

    try
    {
        mTfListen.lookupTransform("world",FrameIdCurr, ros::Time(0), Tf_W_Curr);
        mTfListen.lookupTransform("world",FrameIdMatched, ros::Time(0), Tf_W_Matched);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }

    cv::Mat TCurr = mpCurrentKF->GetPoseInverse();
    cv::Mat TMatch = mpMatchedKF->GetPoseInverse();

    tf::Point PTfCurr{params::vis::mfScaleFactor*((double)(TCurr.at<float>(0,3))),params::vis::mfScaleFactor*((double)(TCurr.at<float>(1,3))),params::vis::mfScaleFactor*((double)(TCurr.at<float>(2,3)))};
    tf::Point PTfMatch{params::vis::mfScaleFactor*((double)(TMatch.at<float>(0,3))),params::vis::mfScaleFactor*((double)(TMatch.at<float>(1,3))),params::vis::mfScaleFactor*((double)(TMatch.at<float>(2,3)))};

    PTfCurr = Tf_W_Curr*PTfCurr;
    PTfMatch = Tf_W_Matched*PTfMatch;

    geometry_msgs::Point PCurr;
    geometry_msgs::Point PMatch;

    tf::pointTFToMsg(PTfCurr,PCurr);
    tf::pointTFToMsg(PTfMatch,PMatch);

    mMapMatchEdgeMsg.points.push_back(PCurr);
    mMapMatchEdgeMsg.points.push_back(PMatch);

    mPubMarker.publish(mMapMatchEdgeMsg);
}

void MapMatcher::ClearLoopEdges()
{
    mMapMatchEdgeMsg.action = 3;
    mPubMarker.publish(mMapMatchEdgeMsg);
    mMapMatchEdgeMsg.action = visualization_msgs::Marker::ADD;
}

void MapMatcher::InsertKF(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexKfInQueue);

    mlKfInQueue.push_back(pKF);
}

void MapMatcher::InsertBKF(bkfptr pBKF)
{
    unique_lock<mutex> lock(mMutexBKfInQueue);

    mlBKfsInQueue.push_back(pBKF);
}

void MapMatcher::EraseKFs(vector<kfptr> vpKFs)
{
    unique_lock<mutex> lock(mMutexKfInQueue);

    for(vector<kfptr>::iterator vit=vpKFs.begin();vit!=vpKFs.end();++vit)
    {
        kfptr pKF = *vit;
        std::list<kfptr>::iterator lit = find(mlKfInQueue.begin(),mlKfInQueue.end(),pKF);
        if(lit!=mlKfInQueue.end()) mlKfInQueue.erase(lit);
    }
}

int MapMatcher::GetNumKFsinQueue()
{
    unique_lock<mutex> lock(mMutexKfInQueue,defer_lock);
    if(lock.try_lock())
    {
        return mlKfInQueue.size();
    }
    else
        return -1;
}

bool MapMatcher::CheckKfQueue()
{
    unique_lock<mutex> lock(mMutexKfInQueue);

    return (!mlKfInQueue.empty());
}

bool MapMatcher::CheckBKfQueue()
{
    unique_lock<mutex> lock(mMutexBKfInQueue);

    return (!mlBKfsInQueue.empty());
}


}
