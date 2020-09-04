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

#include <cslam/Mapping.h>

namespace cslam {

LocalMapping::LocalMapping(ccptr pCC, mapptr pMap, dbptr pDB, viewptr pViewer = nullptr)
    : mpCC(pCC),mpKFDB(pDB),
      mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
      mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true),
      mClientId(pCC->mClientId),
      mpViewer(pViewer)
{
    mAddedKfs = 0;
    mCulledKfs = 0;

    mCountKFs = 0;
}

LocalMapping::LocalMapping(ccptr pCC, bmapptr pBMap, bdbptr pBDB, viewptr pViewer= nullptr)
    :mpCC(pCC),mpBFKsDB(pBDB),
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpBMap(pBMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptBundledKeyFrames(true),
    mClientId(pCC->mClientId),
    mpViewer(pViewer)
{
    mAddedBKfs = 0;
    mCulledBKfs = 0;

    mCountBKFs = 0;      
}

void LocalMapping::RunClient()
{
    mbFinished = false;

    while(1)
    {

        SetAcceptBundledKeyFrames(false);
       
        // Check if there are bundledkeyframes in the queue
        if(CheckNewBundledKeyFrames())
        {
          
            // Get Communicator Mutex -> Comm cannot publish. Assure no publishing whilst changing data
            if(params::sys::mbStrictLock) while(!mpCC->LockMapping()){
                usleep(params::timings::miLockSleep);
            }
   
            // BoW conversion and insertion in Map
            ProcessNewBundledKeyFrames();
          

            //Visualize
            //todo uncomment
            // if(params::vis::mbActive)
            //     if(mpBMap->GetMaxKFid() > 0)
            //         mpViewer->DrawMap(mpBMap);
            //emd todo

            // Check recent MapPoints

            MapPointCullingClient();

            if(!CheckNewBundledKeyFrames())
            {
                // cout<<"SearchInNeighbors enter"<<endl;
                // long temp_time = long(mpCurrentBundledKeyFrames->mTimeStamp*100 - 140363650000);
                // cout<<"第"<<mpCurrentBundledKeyFrames->mId.first<<"帧时间戳 :"<<temp_time<<endl;
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
               
                //cout<<"SearchInNeighbors success"<<endl;
            }

            mbAbortBA = false;

            //Map Forgetting
            while(!mpBMap->LockBMapUpdate()){usleep(params::timings::miLockSleep);}

            mpBMap->MapTrimming(mpCurrentBundledKeyFrames);

            mpBMap->UnLockBMapUpdate();

            if(!CheckNewBundledKeyFrames() && !stopRequested())
            {
                // Local BA
                if(mpBMap->BundledKeyFramesInMap()>2)
                {
                    Optimizer::LocalBundleAdjustmentClient(mpCurrentBundledKeyFrames,&mbAbortBA,mpBMap,mClientId);
                    //cout<<"LocalBundleAdjustmentClient success"<<endl;
                }
                    
            }

            if(params::sys::mbStrictLock)
                mpCC->UnLockMapping();
        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(params::timings::client::miMappingRate);
            }
            if(CheckFinish())
                break;
        }
      
        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptBundledKeyFrames(true);
    
        if(CheckFinish())
            break;
       
        usleep(params::timings::client::miMappingRate); 
    }
    SetFinish();
}

void LocalMapping::RunServer()
{
    // while(1)
    // {
    //     // Check if there are keyframes in the queue
    //     if(CheckNewKeyFrames())
    //     {

    //         //cout <<"线程3:进入Mapping " <<endl;
    //         #ifdef LOGGING
    //         mpCC->mpLogger->SetMapping(__LINE__,mClientId);
    //         #endif

    //         while(!mpCC->LockMapping()){usleep(params::timings::miLockSleep);}

    //         if(mpCC->mbOptActive) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m LocalMapping::Run(...): Optimization active - LocalMapping should be locked" << endl;

    //         #ifdef LOGGING
    //         mpCC->mpLogger->SetMapping(__LINE__,mClientId);
    //         #endif

    //         while(!mpMap->LockMapUpdate()){usleep(params::timings::miLockSleep);}

    //         #ifdef LOGGING
    //         mpCC->mpLogger->SetMapping(__LINE__,mClientId);
    //         #endif

    //         // pop KF from queue
    //         //cout <<"线程3:Mapping ProcessNewKeyFrame" <<endl;
    //         ProcessNewKeyFrame();
    //         // cout <<"++++第 "<<(mpCurrentKeyFrame->mId.first)<< " NewKeyFrames 进入 runServer" <<endl;
    //         //Visualize
    //         mpViewer->DrawMap(mpMap);

    //         // Check recent MapPoints
    //         MapPointCullingServer();

    //         if(!CheckNewKeyFrames())
    //         {
    //             // Find more matches in neighbor keyframes and fuse point duplications
    //            // cout <<"线程3:Mapping SearchInNeighbors" <<endl;
    //             SearchInNeighbors();
    //         }

    //         // Check redundant local Keyframes
    //         if(params::mapping::mfRedundancyThres < 1.0 && !CheckNewKeyFrames())
    //         {
    //             int temp = mpMap->mmpKeyFrames.size();
    //             KeyFrameCullingV3();
    //             cout << "进入culling" << (count++) << "次，删除" <<temp - mpMap->mmpKeyFrames.size()<<"个KF"<<endl;
    //         }

    //         // FIXME:
    //         mpLoopFinder->InsertKF(mpCurrentKeyFrame);
    //         mpMapMatcher->InsertKF(mpCurrentKeyFrame);

    //         mpKFDB->add(mpCurrentKeyFrame);

    //         mpMap->ClearBadMPs();

    //         mpMap->UnLockMapUpdate();

    //         mpCC->UnLockMapping();

    //         #ifdef LOGGING
    //         mpCC->mpLogger->SetMapping(__LINE__,mClientId);
    //         #endif
    //     }
    //     else
    //     {
    //         #ifdef LOGGING
    //         mpCC->mpLogger->SetMapping(__LINE__,mClientId);
    //         #endif
    //     }

    //     ResetIfRequested();

    //     usleep(params::timings::server::miMappingRate);
    // }
}

void LocalMapping::InsertKeyFrame(kfptr pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
}

void LocalMapping::InsertBundledKeyFrames(bkfptr pBKFs)
{
    unique_lock<mutex> lock(mMutexNewBKFs);
    mlNewBundledKeyFrames.push_back(pBKFs);
}

bool LocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

bool LocalMapping::CheckNewBundledKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewBKFs);
    return(!mlNewBundledKeyFrames.empty());
}


void LocalMapping::ProcessNewBundledKeyFrames()
{
    {
        unique_lock<mutex> lock(mMutexNewBKFs);
        mpCurrentBundledKeyFrames = mlNewBundledKeyFrames.front();
        mlNewBundledKeyFrames.pop_front();
    }

    if(mpCC->mSysState == eSystemState::SERVER)
    {
        ++mAddedBKfs;
        ++mCountBKFs;

        vector<mpptr> vpMPs = mpCurrentBundledKeyFrames->GetMapPointMatches();
        for(vector<mpptr>::iterator vit = vpMPs.begin();vit!=vpMPs.end();++vit)
        {
            mpptr pMPi = *vit;

            if(!pMPi || pMPi->isBad())
                continue;

            if(pMPi->mInsertedWithBKFs == -1)
            {
                pMPi->mInsertedWithBKFs = mCountBKFs;
                mlpRecentAddedMapPoints.push_back(pMPi);
            }
        }

        mlpRecentAddedBKFs.push_back(mpCurrentBundledKeyFrames);
        if(mlpRecentAddedBKFs.size() > params::mapping::miNumRecentKFs)
            mlpRecentAddedBKFs.pop_front();
    }
    else if(mpCC->mSysState == eSystemState::CLIENT)
    {
        // Compute Bags of Words structures
        mpCurrentBundledKeyFrames->ComputeBoW();

        // Associate MapPoints to the new keyframe and update normal and descriptor
        const vector<mpptr> vpMapPointMatches = mpCurrentBundledKeyFrames->GetMapPointMatches();
        //add mappoint property for which mappoint created by tracklocalbmap
        for(size_t i=0; i<vpMapPointMatches.size(); i++)
        {
            mpptr pMP = vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!pMP->IsInBundledKeyFrames(mpCurrentBundledKeyFrames))
                    {
                        pMP->AddBKFsObervation(mpCurrentBundledKeyFrames, i,mpCurrentBundledKeyFrames->cameraNum);
                        pMP->UpdateNormalAndDepthPlus();
                        pMP->ComputeDistinctiveDescriptors();
                    }
                    else // this can only happen for new stereo points inserted by the Tracking - remnant from ORB_SLAM, could probably be removed
                    {
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }
                }
            }
        }

        // Update links in the Covisibility Graph
        mpCurrentBundledKeyFrames->UpdateConnections();

        // Insert BundledKeyframes in BMap
        mpBMap->AddBundledKeyFrames(mpCurrentBundledKeyFrames);
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m LocalMapping::ProcessNewKeyFrame(): invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }
}



void LocalMapping::ProcessNewKeyFrame()//对照一下是否区分单双目？@@cjh
{
    
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    if(mpCC->mSysState == eSystemState::SERVER)
    {
        ++mAddedKfs;
        ++mCountKFs;

        vector<mpptr> vpMPs = mpCurrentKeyFrame->GetMapPointMatches();
        for(vector<mpptr>::iterator vit = vpMPs.begin();vit!=vpMPs.end();++vit)
        {
            mpptr pMPi = *vit;

            if(!pMPi || pMPi->isBad())
                continue;

            if(pMPi->mInsertedWithKF == -1)
            {
                pMPi->mInsertedWithKF = mCountKFs;
                mlpRecentAddedMapPoints.push_back(pMPi);
            }
        }

        mlpRecentAddedKFs.push_back(mpCurrentKeyFrame);
        if(mlpRecentAddedKFs.size() > params::mapping::miNumRecentKFs)
            mlpRecentAddedKFs.pop_front();
    }
    else if(mpCC->mSysState == eSystemState::CLIENT)
    {
        // Compute Bags of Words structures
        mpCurrentKeyFrame->ComputeBoW();

        // Associate MapPoints to the new keyframe and update normal and descriptor
        const vector<mpptr> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

        for(size_t i=0; i<vpMapPointMatches.size(); i++)
        {
            mpptr pMP = vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))
                    {
                        pMP->AddObservation(mpCurrentKeyFrame, i);
                        pMP->UpdateNormalAndDepth();
                        pMP->ComputeDistinctiveDescriptors();
                    }
                    else // this can only happen for new stereo points inserted by the Tracking - remnant from ORB_SLAM, could probably be removed
                    {
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }
                }
            }
        }

        // Update links in the Covisibility Graph
        mpCurrentKeyFrame->UpdateConnections();

        // Insert Keyframe in Map
        mpMap->AddKeyFrame(mpCurrentKeyFrame);
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m LocalMapping::ProcessNewKeyFrame(): invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }
}

void LocalMapping::CreateNewMapPoints()
{
    // // Retrieve neighbor keyframes in covisibility graph
    // int nn=10; //对于单双目，用于三角化的观测数不一样
    // if(mbMonocular)
    //     nn = 20;//单目不一样，为20 这里双目也改为20，双目本来为10
    // const vector<kfptr> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    // ORBmatcher matcher(0.6,false);

    // cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
    // cv::Mat Rwc1 = Rcw1.t();
    // cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
    // cv::Mat Tcw1(3,4,CV_32F);
    // Rcw1.copyTo(Tcw1.colRange(0,3));
    // tcw1.copyTo(Tcw1.col(3));
    // cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    // const float &fx1 = mpCurrentKeyFrame->fx;
    // const float &fy1 = mpCurrentKeyFrame->fy;
    // const float &cx1 = mpCurrentKeyFrame->cx;
    // const float &cy1 = mpCurrentKeyFrame->cy;
    // const float &invfx1 = mpCurrentKeyFrame->invfx;
    // const float &invfy1 = mpCurrentKeyFrame->invfy;

    // const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

    // int nnew=0;//三角化成功的点数

    // // Search matches with epipolar restriction and triangulate
    // for(size_t i=0; i<vpNeighKFs.size(); i++)
    // {
    //     if(i>0 && CheckNewKeyFrames())
    //         return;

    //     kfptr pKF2 = vpNeighKFs[i];

    //     // Check first that baseline is not too short
    //     cv::Mat Ow2 = pKF2->GetCameraCenter();
    //     cv::Mat vBaseline = Ow2-Ow1;
    //     const float baseline = cv::norm(vBaseline);

    //     //双目需要判断是不是相机之间的运动大于了相机基线的长度
    //     if(!mbMonocular)
    //     {
    //         if(baseline < pKF2->mb) 
    //             continue;
    //     }
    //     else
    //     {
    //         const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
    //         const float ratioBaselineDepth = baseline/medianDepthKF2;

    //         if(ratioBaselineDepth<0.01)
    //             continue;
    //     }
    //     // const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
    //     // const float ratioBaselineDepth = baseline/medianDepthKF2;
    //     // if(ratioBaselineDepth<0.01)
    //     //     continue;

    //     // Compute Fundamental Matrix
    //     cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

    //     // Search matches that fullfil epipolar constraint
    //     vector<pair<size_t,size_t> > vMatchedIndices;
    //     matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

    //     cv::Mat Rcw2 = pKF2->GetRotation();
    //     cv::Mat Rwc2 = Rcw2.t();
    //     cv::Mat tcw2 = pKF2->GetTranslation();
    //     cv::Mat Tcw2(3,4,CV_32F);
    //     Rcw2.copyTo(Tcw2.colRange(0,3));
    //     tcw2.copyTo(Tcw2.col(3));

    //     const float &fx2 = pKF2->fx;
    //     const float &fy2 = pKF2->fy;
    //     const float &cx2 = pKF2->cx;
    //     const float &cy2 = pKF2->cy;
    //     const float &invfx2 = pKF2->invfx;
    //     const float &invfy2 = pKF2->invfy;

    //     // Triangulate each match
    //     const int nmatches = vMatchedIndices.size();
    //     for(int ikp=0; ikp<nmatches; ikp++)
    //     {
    //         const int &idx1 = vMatchedIndices[ikp].first;
    //         const int &idx2 = vMatchedIndices[ikp].second;

    //         const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
    //         const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
    //         bool bStereo1 = kp1_ur>=0;

    //         const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
    //         const float kp2_ur = pKF2->mvuRight[idx2];
    //         bool bStereo2 = kp2_ur>=0;
    //         ////////////////////////////////////////

    //         // Check parallax between rays
    //         cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
    //         cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

    //         cv::Mat ray1 = Rwc1*xn1;
    //         cv::Mat ray2 = Rwc2*xn2;
    //         const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

    //         //float cosParallaxStereo = cosParallaxRays+1;
    //         float cosParallaxStereo = cosParallaxRays+1;
    //         float cosParallaxStereo1 = cosParallaxStereo;
    //         float cosParallaxStereo2 = cosParallaxStereo;

    //         if(bStereo1)
    //             cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
    //         else if(bStereo2)
    //             cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

    //         cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

    //         cv::Mat x3D;
    //         // 新增　
    //         if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 &&(bStereo1 || bStereo2 || cosParallaxRays<0.9998))
    //         {
    //             // Linear Triangulation Method
    //             cv::Mat A(4,4,CV_32F);
    //             A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
    //             A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
    //             A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
    //             A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

    //             cv::Mat w,u,vt;
    //             cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

    //             x3D = vt.row(3).t();

    //             if(x3D.at<float>(3)==0)
    //                 continue;

    //             // Euclidean coordinates
    //             x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

    //         }
    //         else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
    //         {
    //             x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);                
    //         }
    //         else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
    //         {
    //             x3D = pKF2->UnprojectStereo(idx2);
    //         }
    //         else
    //             continue; //No stereo and very low parallax

    //         cv::Mat x3Dt = x3D.t();

    //         //Check triangulation in front of cameras
    //         float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
    //         if(z1<=0)
    //             continue;

    //         float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
    //         if(z2<=0)
    //             continue;

    //         //Check reprojection error in first keyframe
    //         const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
    //         const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
    //         const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
    //         const float invz1 = 1.0/z1;

    //         if(!bStereo1)
    //         {
    //             float u1 = fx1*x1*invz1+cx1;
    //             float v1 = fy1*y1*invz1+cy1;
    //             float errX1 = u1 - kp1.pt.x;
    //             float errY1 = v1 - kp1.pt.y;
    //             if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
    //                 continue;
    //         }
    //         else
    //         {
    //             float u1 = fx1*x1*invz1+cx1;
    //             float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
    //             float v1 = fy1*y1*invz1+cy1;
    //             float errX1 = u1 - kp1.pt.x;
    //             float errY1 = v1 - kp1.pt.y;
    //             float errX1_r = u1_r - kp1_ur;
    //             if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
    //                 continue;
    //         }

    //         //Check reprojection error in second keyframe
    //         const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
    //         const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
    //         const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
    //         const float invz2 = 1.0/z2;

    //         if(!bStereo2)
    //         {
    //             float u2 = fx2*x2*invz2+cx2;
    //             float v2 = fy2*y2*invz2+cy2;
    //             float errX2 = u2 - kp2.pt.x;
    //             float errY2 = v2 - kp2.pt.y;
    //             if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
    //                 continue;
    //         }
    //         else
    //         {
    //             float u2 = fx2*x2*invz2+cx2;
    //             float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
    //             float v2 = fy2*y2*invz2+cy2;
    //             float errX2 = u2 - kp2.pt.x;
    //             float errY2 = v2 - kp2.pt.y;
    //             float errX2_r = u2_r - kp2_ur;
    //             if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
    //                 continue;
    //         }

    //         //Check scale consistency
    //         cv::Mat normal1 = x3D-Ow1;
    //         float dist1 = cv::norm(normal1);

    //         cv::Mat normal2 = x3D-Ow2;
    //         float dist2 = cv::norm(normal2);

    //         if(dist1==0 || dist2==0)
    //             continue;

    //         const float ratioDist = dist2/dist1;
    //         const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

    //         if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
    //             continue;

    //         // Triangulation is succesfull
    //         //mpptr pMP{new MapPoint(x3D,mpCurrentKeyFrame,mpMap,mClientId,mpComm,mpCC->mSysState,-1)};

    //         pMP->AddObservation(mpCurrentKeyFrame,idx1);
    //         pMP->AddObservation(pKF2,idx2);

    //         mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
    //         pKF2->AddMapPoint(pMP,idx2);

    //         pMP->ComputeDistinctiveDescriptors();

    //         pMP->UpdateNormalAndDepth();

    //         mpMap->AddMapPoint(pMP);
    //         mlpRecentAddedMapPoints.push_back(pMP);

    //         nnew++;
    //     }
    // }
}

// void LocalMapping::SearchInNeighbors()
// {

//     // Retrieve neighbor keyframes
//     int nn = 10; //双目找10帧，单目找20帧 TODO: ORB双目是１０
//     if(mbMonocular)
//         nn=20;
//     const vector<kfptr> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
//     vector<kfptr> vpTargetKFs;
//     for(vector<kfptr>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
//     {
//         kfptr pKFi = *vit;
//         if(pKFi->isBad() || pKFi->mFuseTargetForKF == mpCurrentKeyFrame->mId)
//             continue;
//         vpTargetKFs.push_back(pKFi);
//         pKFi->mFuseTargetForKF = mpCurrentKeyFrame->mId;

//         // Extend to some second neighbors
//         const vector<kfptr> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
//         for(vector<kfptr>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
//         {
//             kfptr pKFi2 = *vit2;
//             if(pKFi2->isBad() || pKFi2->mFuseTargetForKF==mpCurrentKeyFrame->mId || pKFi2->mId==mpCurrentKeyFrame->mId)
//                 continue;
//             vpTargetKFs.push_back(pKFi2);
//         }
//     }

//     // Search matches by projection from current KF in target KFs
//     ORBmatcher matcher;
//     vector<mpptr> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
//     for(vector<kfptr>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
//     {
//         kfptr pKFi = *vit;

//         matcher.Fuse(pKFi,vpMapPointMatches);
//     }

//     // Search matches by projection from target KFs in current KF
//     vector<mpptr> vpFuseCandidates;
//     vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

//     for(vector<kfptr>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
//     {
//         kfptr pKFi = *vitKF;

//         vector<mpptr> vpMapPointsKFi = pKFi->GetMapPointMatches();

//         for(vector<mpptr>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
//         {
//             mpptr pMP = *vitMP;
//             if(!pMP)
//                 continue;
//             if(pMP->isBad() || pMP->mFuseCandidateForKF == mpCurrentKeyFrame->mId)
//                 continue;
//             pMP->mFuseCandidateForKF = mpCurrentKeyFrame->mId;
//             vpFuseCandidates.push_back(pMP);
//         }
//     }

//     matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);

//     // Update points
//     vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
//     for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
//     {
//         mpptr pMP=vpMapPointMatches[i];
//         if(pMP)
//         {
//             if(!pMP->isBad())
//             {
//                 pMP->ComputeDistinctiveDescriptors();
//                 pMP->UpdateNormalAndDepth();
//             }
//         }
//     }

//     // Update connections in covisibility graph
//     mpCurrentKeyFrame->UpdateConnections();
// }

// 检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints
void LocalMapping::SearchInNeighbors()
{

    // Retrieve neighbor keyframes
    int nn = 10; //双目找10帧，单目找20帧 TODO: ORB双目是１０
    if(mbMonocular)
        nn=20;
    const vector<bkfptr> vpNeighBKFs = mpCurrentBundledKeyFrames->GetBestCovisibilityBundledKeyFrames(nn);
    vector<bkfptr> vpTargetBKFs;
  
    for(vector<bkfptr>::const_iterator vit=vpNeighBKFs.begin(), vend=vpNeighBKFs.end(); vit!=vend; vit++)
    {
        bkfptr pBKFi = *vit;
        if(pBKFi->isBad() || pBKFi->mFuseTargetForBKFs == mpCurrentBundledKeyFrames->mId)
            continue;
        vpTargetBKFs.push_back(pBKFi);
        pBKFi->mFuseTargetForBKFs = mpCurrentBundledKeyFrames->mId;

        // Extend to some second neighbors
        const vector<bkfptr> vpSecondNeighBKFs = pBKFi->GetBestCovisibilityBundledKeyFrames(5);
        for(vector<bkfptr>::const_iterator vit2=vpSecondNeighBKFs.begin(), vend2=vpSecondNeighBKFs.end(); vit2!=vend2; vit2++)
        {
            bkfptr pBKFsi2 = *vit2;
            if(pBKFsi2->isBad() || pBKFsi2->mFuseTargetForBKFs==mpCurrentBundledKeyFrames->mId || pBKFsi2->mId==mpCurrentBundledKeyFrames->mId)
                continue;
            vpTargetBKFs.push_back(pBKFsi2);
        }
    }

    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    vector<mpptr> vpMapPointMatches = mpCurrentBundledKeyFrames->GetMapPointMatches();
    for(vector<bkfptr>::iterator vit=vpTargetBKFs.begin(), vend=vpTargetBKFs.end(); vit!=vend; vit++)
    {
        bkfptr pBKFsi = *vit;

        matcher.Fuse(pBKFsi,vpMapPointMatches);
    }
 
    // Search matches by projection from target KFs in current KF
    vector<mpptr> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetBKFs.size()*vpMapPointMatches.size());

    for(vector<bkfptr>::iterator vitBKFs=vpTargetBKFs.begin(), vendBKFs=vpTargetBKFs.end(); vitBKFs!=vendBKFs; vitBKFs++)
    {
        bkfptr pBKFsi = *vitBKFs;

        vector<mpptr> vpMapPointsBKFsi = pBKFsi->GetMapPointMatches();

        for(vector<mpptr>::iterator vitMP=vpMapPointsBKFsi.begin(), vendMP=vpMapPointsBKFsi.end(); vitMP!=vendMP; vitMP++)
        {
            mpptr pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mFuseCandidateForBKFs == mpCurrentBundledKeyFrames->mId)
                continue;
            pMP->mFuseCandidateForBKFs = mpCurrentBundledKeyFrames->mId;
            vpFuseCandidates.push_back(pMP);
        }
    }
 
    matcher.Fuse(mpCurrentBundledKeyFrames,vpFuseCandidates);
    
    // Update points
    vpMapPointMatches = mpCurrentBundledKeyFrames->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        mpptr pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepthPlus();
            }
        }
    }
    
    // Update connections in covisibility graph
    mpCurrentBundledKeyFrames->UpdateConnections();
    
}

cv::Mat LocalMapping::ComputeF12(kfptr &pKF1, kfptr &pKF2)
{
    cv::Mat R1w = pKF1->GetRotation();
    cv::Mat t1w = pKF1->GetTranslation();
    cv::Mat R2w = pKF2->GetRotation();
    cv::Mat t2w = pKF2->GetTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void LocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "++++++Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

bool LocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

void LocalMapping::SetAcceptBundledKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptBundledKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

void LocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        if(mpCC->mSysState == eSystemState::CLIENT)
            usleep(params::timings::client::miMappingRate);
        else if(mpCC->mSysState == eSystemState::SERVER)
            usleep(params::timings::server::miMappingRate);
        else KILLSYS

    }
}

void LocalMapping::ResetIfRequested()
{    
    unique_lock<mutex> lock(mMutexReset);

    if(mbResetRequested)
    {
        mlNewBundledKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
//todo
        // if(mpLoopFinder)
        // {
        //     mpLoopFinder->RequestReset();
        // }
//end todo uncomment
        mbResetRequested=false;
    }
}

void LocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

// void LocalMapping::MapPointCullingClient()
// {
//     // Check Recent Added MapPoints
//     list<mpptr>::iterator lit = mlpRecentAddedMapPoints.begin();
//     const idpair nCurrentKFid = mpCurrentKeyFrame->mId;

//     int nThObs;
//     if(mbMonocular) //TODO: 源码单目是３
//         nThObs = 2;
//     else
//         nThObs = 3;
//     const int cnThObs = nThObs;

//     while(lit!=mlpRecentAddedMapPoints.end())
//     {
//         mpptr pMP = *lit;
//         if(pMP->isBad())// 分不分单双面？@@cjh
//         {
//             lit = mlpRecentAddedMapPoints.erase(lit);
//         }
//         else if(pMP->GetFoundRatio()<0.25f )
//         {
//             pMP->SetBadFlag();
//             lit = mlpRecentAddedMapPoints.erase(lit);
//         }
//         else if(((int)nCurrentKFid.first-(int)pMP->mFirstKfId.first)>=2 && nCurrentKFid.second == pMP->mFirstFrame.second && pMP->Observations()<=cnThObs)
//         {
//             pMP->SetBadFlag();
//             lit = mlpRecentAddedMapPoints.erase(lit);
//         }
//         else if(((int)nCurrentKFid.first-(int)pMP->mFirstKfId.first)>=3)
//             lit = mlpRecentAddedMapPoints.erase(lit);
//         else
//             lit++;
//     }
// }

void LocalMapping::MapPointCullingClient()
{
    // Check Recent Added MapPoints
    list<mpptr>::iterator lit = mlpRecentAddedMapPoints.begin();
    const idpair nCurrentBKFsid = mpCurrentBundledKeyFrames->mId;

    int nThObs;
    if(mbMonocular) //TODO: 源码单目是３
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        mpptr pMP = *lit;
        if(pMP->isBad())// 分不分单双面？@@cjh
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlagBKFs();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentBKFsid.first-(int)pMP->mFirstBKfsId.first)>=2 && nCurrentBKFsid.second == pMP->mFirstFrame.second && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlagBKFs();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentBKFsid.first-(int)pMP->mFirstBKfsId.first)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

void LocalMapping::MapPointCullingServer()
{
    // Check Recent Added MapPoints
    list<mpptr>::iterator lit = mlpRecentAddedMapPoints.begin();

    int nThObs;
    nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        mpptr pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if((mCountKFs - pMP->mInsertedWithKF) >= 3 && pMP->mId.second == mpCC->mClientId && pMP->Observations() <= cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if((mCountKFs - pMP->mInsertedWithKF) >= 3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

void LocalMapping::KeyFrameCullingV3()
{
    //This version: randomly pick a KF and check for redundancy
    kfptr pKFc = mpMap->GetRandKfPtr();
    if(!pKFc)
        return; //safety check

    //we don't check KFs in mlpRecentAddedKFs, since the neighbors will probably not be allowed for culling.
    list<kfptr>::iterator lit1 = std::find(mlpRecentAddedKFs.begin(),mlpRecentAddedKFs.end(),pKFc);
    if(lit1 != mlpRecentAddedKFs.end())
    {
        //give it a second try -- if not successful return to not spend ages in this method.

        pKFc = mpMap->GetRandKfPtr();
            if(!pKFc)
                return; //safety check

        lit1 = std::find(mlpRecentAddedKFs.begin(),mlpRecentAddedKFs.end(),pKFc);
        if(lit1 != mlpRecentAddedKFs.end())
            return;
    }

    if(mspKFsCheckedForCulling.count(pKFc))
        return;
    else
        mspKFsCheckedForCulling.insert(pKFc);

    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    vector<kfptr> vpLocalKeyFrames = pKFc->GetVectorCovisibleKeyFrames();

    for(vector<kfptr>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        kfptr pKF = *vit;
        if(pKF->mId.first==0) //don't cull 0, since it's the origin, and also not one, because this was the other KF used for initialization. The systen won't experience problems if culling 1, nevetheless we don't do it.
            continue;

        list<kfptr>::iterator lit2 = std::find(mlpRecentAddedKFs.begin(),mlpRecentAddedKFs.end(),pKF);
        if(lit2 != mlpRecentAddedKFs.end())
            continue;
        const vector<mpptr> vpMapPoints = pKF->GetMapPointMatches();

        const int thObs=3;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            mpptr pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    bool isStereo = true; // 新增　参考ORB
                    if(isStereo)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }
                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<kfptr, size_t> observations = pMP->GetObservations();
                        int nObs=0;
                        for(map<kfptr, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            kfptr pKFi = mit->first;

                            if(pKFi->isBad()) continue;

                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }
        if(nRedundantObservations>params::mapping::mfRedundancyThres*nMPs)
        {
            pKF->SetBadFlag();
            ++mCulledKfs;
        }
    }
}

void LocalMapping::ClearCovGraph(size_t MapId)
{
    mpViewer->ClearCovGraph(MapId);
}

cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
            v.at<float>(2),               0,-v.at<float>(0),
            -v.at<float>(1),  v.at<float>(0),              0);
}

//在client的local mapping中没有加入keyframesculling， 而是在server中加入的，为了测试，client的效果，先加入-------------
// 关键帧剔除,在Covisibility Graph中的关键帧，其90%以上的MapPoints能被其他关键帧（至少3个）观测到，则认为该关键帧为冗余关键帧。
void LocalMapping::KeyFrameCulling() 
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points

    // STEP1：根据Covisibility Graph提取当前帧的共视关键帧 (所有)
    vector<kfptr> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    // 对所有的局部关键帧进行遍历 ; 这里的局部关键帧就理解为当前关键帧的共视帧
    for(vector<kfptr>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        kfptr pKF = *vit;
        if(pKF->mId.first==0)
            continue;
        // STEP2：提取每个共视关键帧的MapPoints
        const vector<mpptr> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;                       // 接下来程序中使用到的循环变量,记录了某个点的被观测次数 (其实这里初始化是没有意义的)
        const int thObs=nObs;               // 观测阈值,默认为3
        int nRedundantObservations=0;       // 冗余观测地图点的计数器
                                            // 不是说我从地图点中得到了其观测数据我就信了,这里还要求这个地图点在当前关键帧和在邻接关键帧中的特征尺度变化不太大才可以
                                            // 认为这个地图点被"冗余观测"了
        int nMPs=0;                         // 计数器,参与到检测的地图点的总数目

        // STEP3：遍历该局部关键帧的MapPoints，判断是否90%以上的MapPoints能被其它关键帧（至少3个）观测到
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            mpptr pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        // 对于双目，仅考虑近处的MapPoints，不超过mbf * 35 / fx 
                        // 不过配置文件中给的是近点的定义是 40 * fx
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++; 
                    // MapPoints至少被三个关键帧观测到
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const map<kfptr, size_t> observations = pMP->GetObservations();
                        // 判断该MapPoint是否同时被三个关键帧观测到
                        int nObs=0;
                        // 遍历当前这个邻接关键帧上的地图点的所有观测信息
                        for(map<kfptr, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            kfptr pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                            // Scale Condition 
                            // 尺度约束，要求MapPoint在该局部关键帧的特征尺度大于（或近似于）其它关键帧的特征尺度
                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                // 已经找到三个同尺度的关键帧可以观测到该MapPoint，不用继续找了
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        // 该MapPoint至少被三个关键帧观测到
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        // STEP4：该局部关键帧90%以上的MapPoints能被其它关键帧（至少3个）观测到，则认为是冗余关键帧
        if(nRedundantObservations>0.9*nMPs)
            // 剔除的时候就设置一个 bad flag 就可以了
            pKF->SetBadFlag();
    }
}
} //end ns
