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

#include <cslam/ClientHandler.h>

namespace cslam {

ClientHandler::ClientHandler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, vocptr pVoc, dbptr pDB, mapptr pMap, size_t ClientId, uidptr pUID, eSystemState SysState, const string &strCamFile, viewptr pViewer)
    : mpVoc(pVoc),mpKFDB(pDB),mpMap(pMap),
      mNh(Nh),mNhPrivate(NhPrivate),
      mClientId(ClientId), mpUID(pUID), mSysState(SysState),
      mstrCamFile(strCamFile),
      mpViewer(pViewer),mbReset(false)
{
    if(mpVoc == nullptr || mpKFDB == nullptr || mpMap == nullptr || (mpUID == nullptr && mSysState == eSystemState::SERVER))
    {
        cout << ("In \" ClientHandler::ClientHandler(...)\": nullptr exception") << endl;
        throw estd::infrastructure_ex();
    }

    mpMap->msuAssClients.insert(mClientId);

    mg2oS_wcurmap_wclientmap = g2o::Sim3(); //identity transformation

    if(mSysState == eSystemState::CLIENT)
    {
		
		
		if(ClientId==1)
        {
            mSubOP=mNh.subscribe<ccmslam_msgs::ObjectPosition> ("ObjectPositionTopicName",10,boost::bind(&ClientHandler::ObejectPositionCb,this,_1));//zmf add
        }
        std::string TopicNameCamSub;

        mNhPrivate.param("TopicNameCamSub",TopicNameCamSub,string("nospec"));
        // mSubCam = mNh.subscribe<sensor_msgs::Image>(TopicNameCamSub,10,boost::bind(&ClientHandler::CamImgCb,this,_1));

        cout << "stereo Camera Input topic: " << TopicNameCamSub << endl;
    }
}
//multi
ClientHandler::ClientHandler(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, vocptr pVoc, bdbptr pBDB, bmapptr pBMap, size_t ClientId, uidptr pUID, eSystemState SysState, const string &strCamFile, viewptr pViewer)
    : mpVoc(pVoc),mpBKFDB(pBDB),mpBMap(pBMap),
      mNh(Nh),mNhPrivate(NhPrivate),
      mClientId(ClientId), mpUID(pUID), mSysState(SysState),
      mstrCamFile(strCamFile),
      mpViewer(pViewer),mbReset(false)
{
    if(mpVoc == nullptr || mpBKFDB == nullptr || mpBMap == nullptr || (mpUID == nullptr && mSysState == eSystemState::SERVER))
    {
        cout << ("In \" ClientHandler::ClientHandler(...)\": nullptr exception") << endl;
        throw estd::infrastructure_ex();
    }

    mpBMap->msuAssClients.insert(mClientId);

    mg2oS_wcurmap_wclientmap = g2o::Sim3(); //identity transformation

    if(mSysState == eSystemState::CLIENT)
    {
		
		//TODO
		// if(ClientId==1)
        // {
        //     mSubOP=mNh.subscribe<ccmslam_msgs::ObjectPosition> ("ObjectPositionTopicName",10,boost::bind(&ClientHandler::ObejectPositionCb,this,_1));//zmf add
        // }
        //ENDTO
        std::string TopicNameCamSub;

        mNhPrivate.param("TopicNameCamSub",TopicNameCamSub,string("nospec"));
        // mSubCam = mNh.subscribe<sensor_msgs::Image>(TopicNameCamSub,10,boost::bind(&ClientHandler::CamImgCb,this,_1));

        cout << "multi Camera Input topic: " << TopicNameCamSub << endl;
    }
}

// //开始从这里修改，加上右目图像的信息
// void ClientHandler::CamImgCb(sensor_msgs::ImageConstPtr msgLeft, sensor_msgs::ImageConstPtr msgRight)
// {
// //这里需要修改
//     // Copy the ros image message to cv::Mat.
//     cv_bridge::CvImageConstPtr cv_ptrLeft;
//     try
//     {
//         cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     cv_bridge::CvImageConstPtr cv_ptrRight;
//     try
//     {
//         cv_ptrRight = cv_bridge::toCvShare(msgRight);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     // Check reset
//     {
//         unique_lock<mutex> lock(mMutexReset);
//         if(mbReset)
//         {
//             mpTracking->Reset();
//             mbReset = false;
//         }
//     }

//     //12.19，加入对Euroc数据集的矫正
//     int do_retify = 0;
//     if (do_retify)
//     {
//         //cout<<"矫正进行+++++++++++++++++++++++"<<endl;
//         cv::FileStorage fsSettings(mstrCamFile, cv::FileStorage::READ);
//         cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
//         cv::Mat M1l,M2l,M1r,M2r;
//         fsSettings["LEFT.K"] >> K_l;
//         fsSettings["RIGHT.K"] >> K_r;

//         fsSettings["LEFT.P"] >> P_l;
//         fsSettings["RIGHT.P"] >> P_r;

//         fsSettings["LEFT.R"] >> R_l;
//         fsSettings["RIGHT.R"] >> R_r;

//         fsSettings["LEFT.D"] >> D_l;
//         fsSettings["RIGHT.D"] >> D_r;

//         int rows_l = fsSettings["LEFT.height"];
//         int cols_l = fsSettings["LEFT.width"];
//         int rows_r = fsSettings["RIGHT.height"];
//         int cols_r = fsSettings["RIGHT.width"];

//         if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
//                 rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
//         {
//             cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            
//         }
//         cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
//         cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

//         cv::Mat imLeft, imRight;
//         cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
//         cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
//         mpTracking->GrabImageStereo(imLeft, imRight, cv_ptrLeft->header.stamp.toSec());
//     }
//     else
//     {
//         mpTracking->GrabImageStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());
//     }


// }


#ifdef LOGGING
void ClientHandler::InitializeThreads(boost::shared_ptr<estd::mylog> pLogger)
#else
void ClientHandler::InitializeThreads()
#endif
{
    #ifdef LOGGING
    this->InitializeCC(pLogger);
    #else
    this->InitializeCC();
    #endif

    if(mSysState == eSystemState::CLIENT)
    {
        this->InitializeClient();
    }
    else if(mSysState == eSystemState::SERVER)
    {
        this->InitializeServer();
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::InitializeThreads(): invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }
}

// #ifdef LOGGING
// void ClientHandler::InitializeCC(boost::shared_ptr<mylog> pLogger)
// #else
// void ClientHandler::InitializeCC()
// #endif
// {
//     std::stringstream* ss;

//     mpCC.reset(new CentralControl(mNh,mNhPrivate,mClientId,mSysState,shared_from_this(),mpUID));

//     if(mSysState == eSystemState::CLIENT)
//     {
//         ss = new stringstream;
//         *ss << "FrameId";
//         mNhPrivate.param(ss->str(),mpCC->mNativeOdomFrame,std::string("nospec"));
//     }
//     else if(mSysState == eSystemState::SERVER)
//     {
//         ss = new stringstream;
//         *ss << "FrameId" << mClientId;
//         mNhPrivate.param(ss->str(),mpCC->mNativeOdomFrame,std::string("nospec"));
//     }
//     else
//     {
//         cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::InitializeThreads(): invalid systems state: " << mpCC->mSysState << endl;
//         throw infrastructure_ex();
//     }

//     if(mpCC->mNativeOdomFrame=="nospec")
//     {
//         ROS_ERROR_STREAM("In \" ServerCommunicator::ServerCommunicator(...)\": bad parameters");
//         throw estd::infrastructure_ex();
//     }

//     {
//         if(mSysState==CLIENT)
//         {
//             cv::FileStorage fSettings(mstrCamFile, cv::FileStorage::READ);

//             float c0t00 = fSettings["Cam0.T00"];
//             float c0t01 = fSettings["Cam0.T01"];
//             float c0t02 = fSettings["Cam0.T02"];
//             float c0t03 = fSettings["Cam0.T03"];
//             float c0t10 = fSettings["Cam0.T10"];
//             float c0t11 = fSettings["Cam0.T11"];
//             float c0t12 = fSettings["Cam0.T12"];
//             float c0t13 = fSettings["Cam0.T13"];
//             float c0t20 = fSettings["Cam0.T20"];
//             float c0t21 = fSettings["Cam0.T21"];
//             float c0t22 = fSettings["Cam0.T22"];
//             float c0t23 = fSettings["Cam0.T23"];
//             float c0t30 = fSettings["Cam0.T30"];
//             float c0t31 = fSettings["Cam0.T31"];
//             float c0t32 = fSettings["Cam0.T32"];
//             float c0t33 = fSettings["Cam0.T33"];
//             mpCC->mT_SC << c0t00,c0t01,c0t02,c0t03,c0t10,c0t11,c0t12,c0t13,c0t20,c0t21,c0t22,c0t23,c0t30,c0t31,c0t32,c0t33;
//         }
//         else
//         {
//             //no mstrCamFile on Server...
//         }
//     }

//     mpMap->mOdomFrame = mpCC->mNativeOdomFrame;
//     mpMap->AddCCPtr(mpCC);

//     #ifdef LOGGING
//     mpCC->mpLogger = pLogger;
//     #endif

//     delete ss;
// }

void ClientHandler::InitializeCC()
{
    std::stringstream* ss;

    mpCC.reset(new CentralControl(mNh,mNhPrivate,mClientId,mSysState,shared_from_this(),mpUID));

    if(mSysState == eSystemState::CLIENT)
    {
        ss = new stringstream;
        *ss << "FrameId";
        mNhPrivate.param(ss->str(),mpCC->mNativeOdomFrame,std::string("nospec"));
    }
    else if(mSysState == eSystemState::SERVER)
    {
        ss = new stringstream;
        *ss << "FrameId" << mClientId;
        mNhPrivate.param(ss->str(),mpCC->mNativeOdomFrame,std::string("nospec"));
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::InitializeThreads(): invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }

    if(mpCC->mNativeOdomFrame=="nospec")
    {
        ROS_ERROR_STREAM("In \" ServerCommunicator::ServerCommunicator(...)\": bad parameters");
        throw estd::infrastructure_ex();
    }

    {
        if(mSysState==CLIENT)
        {
            cv::FileStorage fSettings(mstrCamFile, cv::FileStorage::READ);

            float c0t00 = fSettings["Cam0.T00"];
            float c0t01 = fSettings["Cam0.T01"];
            float c0t02 = fSettings["Cam0.T02"];
            float c0t03 = fSettings["Cam0.T03"];
            float c0t10 = fSettings["Cam0.T10"];
            float c0t11 = fSettings["Cam0.T11"];
            float c0t12 = fSettings["Cam0.T12"];
            float c0t13 = fSettings["Cam0.T13"];
            float c0t20 = fSettings["Cam0.T20"];
            float c0t21 = fSettings["Cam0.T21"];
            float c0t22 = fSettings["Cam0.T22"];
            float c0t23 = fSettings["Cam0.T23"];
            float c0t30 = fSettings["Cam0.T30"];
            float c0t31 = fSettings["Cam0.T31"];
            float c0t32 = fSettings["Cam0.T32"];
            float c0t33 = fSettings["Cam0.T33"];
            mpCC->mT_SC << c0t00,c0t01,c0t02,c0t03,c0t10,c0t11,c0t12,c0t13,c0t20,c0t21,c0t22,c0t23,c0t30,c0t31,c0t32,c0t33;
        }
        else
        {
            //no mstrCamFile on Server...
        }
    }

    mpBMap->mOdomFrame = mpCC->mNativeOdomFrame;
    mpBMap->AddCCPtr(mpCC);

    #ifdef LOGGING
    mpCC->mpLogger = pLogger;
    #endif

    delete ss;
}


// void ClientHandler::InitializeClient()
// {
//     cout << "Client " << mClientId << " --> Initialize Threads" << endl;

//     //+++++ Create Drawers. These are used by the Viewer +++++
//     mpViewer.reset(new Viewer(mpMap,mpCC));
//     usleep(10000);
//     //+++++ Initialize the Local Mapping thread +++++
//     mpMapping.reset(new LocalMapping(mpCC,mpMap,mpKFDB,mpViewer));
//     usleep(10000);
//     //    +++++ Initialize the communication thread +++++
//     mpComm.reset(new Communicator(mpCC,mpVoc,mpMap,mpKFDB));
//     mpComm->SetMapping(mpMapping);
//     usleep(10000);
//     mpMap->SetCommunicator(mpComm);
//     mpMapping->SetCommunicator(mpComm);
//     usleep(10000);
//     //+++++ Initialize the tracking thread +++++
//     //(it will live in the main thread of execution, the one that called this constructor)
//     mpTracking.reset(new Tracking(mpCC, mpVoc, mpViewer, mpMap, mpKFDB, mstrCamFile, mClientId));
//     usleep(10000);
//     mpTracking->SetCommunicator(mpComm);
//     mpTracking->SetLocalMapper(mpMapping);
//     mpViewer->SetTracker(mpTracking);
//     usleep(10000);
//     //Launch Threads
// 	if (mClientId == 0)
//     {
//         mpSkeleton.reset(new Skeleton());//构造函数的参数暂时没有定
//         mpSkeleton->SetCommunicator(mpComm);//传comm的指针过去
//     }
//     usleep(10000);
//     //Should no do that before, a fast system might already use a pointe before it was set -> segfault
//     mptMapping.reset(new thread(&LocalMapping::RunClient,mpMapping));
//     mptComm.reset(new thread(&Communicator::RunClient,mpComm));
//     mptViewer.reset(new thread(&Viewer::RunClient,mpViewer));
// 	if (mClientId == 0)
//     {
//          //cout<<"启动骨架 线程1！!!!!!!!!"<<endl;
//         mptskeleton.reset(new thread(&Skeleton::RunClient,mpSkeleton));
//          //cout<<"启动骨架 线程2！!!!!!!!!"<<endl;

//     }
//     usleep(10000);
// 	cout<<"client 线程全部启动完毕！!!!!!!!!"<<endl;
// }

//MULTI
void ClientHandler::InitializeClient()
{
    cout << "Client (multi-camera system) " << mClientId << " --> Initialize Threads" << endl;

    //+++++ Create Drawers. These are used by the Viewer +++++
   
    mpViewer.reset(new Viewer(mpBMap,mpCC));
    usleep(10000);
    

    //+++++ Initialize the Local Mapping thread +++++
    mpMapping.reset(new LocalMapping(mpCC,mpBMap,mpBKFDB,mpViewer));
    usleep(10000);
    //    +++++ Initialize the communication thread +++++
    mpComm.reset(new Communicator(mpCC,mpVoc,mpBMap,mpBKFDB));
    mpComm->SetMapping(mpMapping);
    usleep(10000);
    mpBMap->SetCommunicator(mpComm);
    mpMapping->SetCommunicator(mpComm);
    usleep(10000);
    //+++++ Initialize the tracking thread +++++
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracking.reset(new Tracking(mpCC, mpVoc, mpViewer, mpBMap, mpBKFDB, mstrCamFile, mClientId));
    usleep(10000);
    mpTracking->SetCommunicator(mpComm);
    mpTracking->SetLocalMapper(mpMapping);
    mpViewer->SetTracker(mpTracking);  
    usleep(10000);
    //Launch Threads
    //todo
	// if (mClientId == 0)
    // {
    //     mpSkeleton.reset(new Skeleton());//构造函数的参数暂时没有定
    //     mpSkeleton->SetCommunicator(mpComm);//传comm的指针过去
    // }
    //usleep(10000);
    //end todo

    //Should no do that before, a fast system might already use a pointe before it was set -> segfault
    mptMapping.reset(new thread(&LocalMapping::RunClient,mpMapping));//todo uncomment
    mptComm.reset(new thread(&Communicator::RunClient,mpComm));  //todo uncomment
    mptViewer.reset(new thread(&Viewer::RunClient,mpViewer));  //do not forget to uncomment
    //todo
	// if (mClientId == 0)
    // {
    //      //cout<<"启动骨架 线程1！!!!!!!!!"<<endl;
    //     mptskeleton.reset(new thread(&Skeleton::RunClient,mpSkeleton));
    //      //cout<<"启动骨架 线程2！!!!!!!!!"<<endl;

    // }
    // usleep(10000);
    //endtodo

	cout<<"client (multi-camera system) 线程全部启动完毕！!!!!!!!!"<<endl;
}

void ClientHandler::InitializeServer()
{
    cout << "Client " << mClientId << " --> Initialize Threads" << endl;

    //+++++ Initialize the Loop Finder thread and launch +++++
    // mpLoopFinder.reset(new LoopFinder(mpCC,mpKFDB,mpVoc,mpMap));
    // mptLoopClosure.reset(new thread(&LoopFinder::Run,mpLoopFinder));
    // usleep(10000);
    //+++++ Initialize the Local Mapping thread +++++
    mpMapping.reset(new LocalMapping(mpCC,mpBMap,mpBKFDB,mpViewer));
    //mpMapping->SetLoopFinder(mpLoopFinder); //tempout
    usleep(10000);
    //+++++ Initialize the communication thread +++++
    mpComm.reset(new Communicator(mpCC,mpVoc,mpBMap,mpBKFDB));
    mpComm->SetMapping(mpMapping);
    usleep(10000);
    mpMapping->SetCommunicator(mpComm);
    mpBMap->SetCommunicator(mpComm);
    usleep(10000);
    //Launch Threads
    //Should not do that before, a fast system might already use a pointer before it was set -> segfault
    mptMapping.reset(new thread(&LocalMapping::RunServer,mpMapping));
    mptComm.reset(new thread(&Communicator::RunServer,mpComm));
    usleep(10000);
    if(mpCC->mpCH == nullptr)
    {
        ROS_ERROR_STREAM("ClientHandler::InitializeThreads()\": mpCC->mpCH is nullptr");
        throw estd::infrastructure_ex();
    }
}

void ClientHandler::ChangeBMap(bmapptr pBMap, g2o::Sim3 g2oS_wnewmap_wcurmap)
{
    mpBMap = pBMap;

    mg2oS_wcurmap_wclientmap = g2oS_wnewmap_wcurmap*mg2oS_wcurmap_wclientmap;
    mpCC->mg2oS_wcurmap_wclientmap = mg2oS_wcurmap_wclientmap;

    bool bLockedComm = mpCC->LockComm(); //should be locked and therefore return false
    //    #ifdef LOGGING
    //    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
    //    #endif
    bool bLockedMapping = mpCC->LockMapping(); //should be locked and therefore return false
    //    #ifdef LOGGING
    //    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
    //    #endif

    if(bLockedComm || bLockedMapping)
    {
        if(bLockedComm) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::ChangeMap(): Comm not locked: " << endl;
        if(bLockedMapping) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::ChangeMap(): Mapping not locked: " << endl;
        throw infrastructure_ex();
    }

    //    #ifdef LOGGING
    //    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
    //    #endif

    mpComm->ChangeBMap(mpBMap);
    mpMapping->ChangeBMap(mpBMap); //tempout
    //mpLoopFinder->ChangeMap(mpMap); //tempout //todo： 自己的回环检测目前版本不需要了

    //    #ifdef LOGGING
    //    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
    //    #endif
}

void ClientHandler::ChangeMap(mapptr pMap, g2o::Sim3 g2oS_wnewmap_wcurmap)
{
    mpMap = pMap;

    mg2oS_wcurmap_wclientmap = g2oS_wnewmap_wcurmap*mg2oS_wcurmap_wclientmap;
    mpCC->mg2oS_wcurmap_wclientmap = mg2oS_wcurmap_wclientmap;

    bool bLockedComm = mpCC->LockComm(); //should be locked and therefore return false
    //    #ifdef LOGGING
    //    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
    //    #endif
    bool bLockedMapping = mpCC->LockMapping(); //should be locked and therefore return false
    //    #ifdef LOGGING
    //    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
    //    #endif

    if(bLockedComm || bLockedMapping)
    {
        if(bLockedComm) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::ChangeMap(): Comm not locked: " << endl;
        if(bLockedMapping) cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m ClientHandler::ChangeMap(): Mapping not locked: " << endl;
        throw infrastructure_ex();
    }

    //    #ifdef LOGGING
    //    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
    //    #endif

    mpComm->ChangeMap(mpMap);
    mpMapping->ChangeMap(mpMap); //tempout
    mpLoopFinder->ChangeMap(mpMap); //tempout

    //    #ifdef LOGGING
    //    mpCC->mpLogger->SetMappingLock(__LINE__,mpCC->mClientId);
    //    #endif
}



void ClientHandler::SetMapMatcher(matchptr pMatch)
{
    mpMapMatcher = pMatch;
    mpComm->SetMapMatcher(mpMapMatcher);
    mpMapping->SetMapMatcher(mpMapMatcher);
}
//处理关键物体位置信息的回调函数  zmf add
void ClientHandler::ObejectPositionCb(ccmslam_msgs::ObjectPositionConstPtr pMsg)
{
    // 这里为终端B接受云端服务器发送过来的27个点的三维坐标
    //相关临时存储变量应该在ClientHandler类里面定义
    this->ObjectPostition.clear();
    this->OPTimeStamp=pMsg->Timestamp;
    this->Client_id=pMsg->Client_ID;
    vector<float>temp(3);
    for(int i=0;i<pMsg->Position.size();i++)
    {
        temp[i%3]=pMsg->Position[i];
        if((i+1)%3==0)
        {
            this->ObjectPostition.push_back(temp);
        }
    }

}


// void ClientHandler::CamImgCb(sensor_msgs::ImageConstPtr msgLeft, sensor_msgs::ImageConstPtr msgRight,sensor_msgs::ImageConstPtr msgdepth_img)
// {
//     // Copy the ros image message to cv::Mat.
//     cv_bridge::CvImageConstPtr cv_ptrLeft;
//     try
//     {
//         cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     cv_bridge::CvImageConstPtr cv_ptrRight;
//     try
//     {
//         cv_ptrRight = cv_bridge::toCvShare(msgRight);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

    
//     cv_bridge::CvImageConstPtr cv_ptrDepth;
//     try
//     {
//         cv_ptrDepth = cv_bridge::toCvShare(msgdepth_img);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }

//     // Check reset
//     {
//         unique_lock<mutex> lock(mMutexReset);
//         if(mbReset)
//         {
//             mpTracking->Reset();
//             mbReset = false;
//         }
//     }
//     //12.19，加入对Euroc数据集的矫正
    
//     //cout<<"矫正进行+++++++++++++++++++++++"<<endl;
//     cv::FileStorage fsSettings(mstrCamFile, cv::FileStorage::READ);
//     cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
//     cv::Mat M1l,M2l,M1r,M2r;
//     fsSettings["LEFT.K"] >> K_l;
//     fsSettings["RIGHT.K"] >> K_r;

//     fsSettings["LEFT.P"] >> P_l;
//     fsSettings["RIGHT.P"] >> P_r;

//     fsSettings["LEFT.R"] >> R_l;
//     fsSettings["RIGHT.R"] >> R_r;

//     fsSettings["LEFT.D"] >> D_l;
//     fsSettings["RIGHT.D"] >> D_r;

//     int rows_l = fsSettings["LEFT.height"];
//     int cols_l = fsSettings["LEFT.width"];
//     int rows_r = fsSettings["RIGHT.height"];
//     int cols_r = fsSettings["RIGHT.width"];

//     if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
//             rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
//     {
//         cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        
//     }
//     cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
//     cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

//     cv::Mat imLeft, imRight;
//     cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
//     cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
//     //zmf 修改
    
//     double timeStampTmp = cv_ptrLeft->header.stamp.toSec();
//     //cout<<"timeStampTmp time: "<<setw(20)<<setprecision(15)<<setiosflags(ios::fixed)<<timeStampTmp<<endl;
//     if (mClientId == 0)
//     {
//         //cv::imshow("左目校正图像",imLeft);
//         //cv::imshow("左目原始图像",cv_ptrLeft->image);
//         //cv::waitKey(0);
//         mpSkeleton->img_write(timeStampTmp,imLeft,cv_ptrDepth->image);//这里只传入了左目 和深度的图像
//     }
        
//     //cout<<"OPTimeStamp="<<this->OPTimeStamp<<"x="<<this->ObjectPostition_x<<"y="<<this->ObjectPostition_y<<"z="<<this->ObjectPostition_z<<endl;
//     mpTracking->GrabImageStereo(imLeft, imRight, timeStampTmp,this->OPTimeStamp,this->ObjectPostition);
//     //mpTracking->GrabImageMonocular(cv_ptrLeft->image,timeStampTmp,this->OPTimeStamp,this->ObjectPostition);

// }

//to do multi camera input do not forget when we get extrinic parameter

//end
void ClientHandler::CamImgCb(sensor_msgs::ImageConstPtr msg0, sensor_msgs::ImageConstPtr msg1) //,sensor_msgs::ImageConstPtr msgdepth_img
{
    //todo
    cout<<"++++++++++++++++++++++++++++++++++++++++++++++++++++"<<endl;
    cout<<"start to grab image "<<endl;
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr0;
    try
    {
        cv_ptr0 = cv_bridge::toCvShare(msg0);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptr1;
    try
    {
        cv_ptr1 = cv_bridge::toCvShare(msg1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // cv_bridge::CvImageConstPtr cv_ptrDepth;
    // try
    // {
    //     cv_ptrDepth = cv_bridge::toCvShare(msgdepth_img);
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    //     return;
    // }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracking->Reset();
            mbReset = false;
        }
    }


    //12.19，加入对Euroc数据集的矫正
    
    //cout<<"矫正进行+++++++++++++++++++++++"<<endl;
    cv::FileStorage fsSettings(mstrCamFile, cv::FileStorage::READ);
    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    cv::Mat M1l,M2l,M1r,M2r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    int cameraNum = 2;
    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        
    }
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
  
    vector<cv::Mat> vImagesRect(cameraNum);

    cv::remap(cv_ptr0->image,vImagesRect[0],M1l,M2l,cv::INTER_LINEAR);
    cv::remap(cv_ptr1->image,vImagesRect[1],M1r,M2r,cv::INTER_LINEAR);
    //zmf 修改
    
    double timeStampTmp = cv_ptr0->header.stamp.toSec();
    
    //cout<<"timeStampTmp time: "<<setw(20)<<setprecision(15)<<setiosflags(ios::fixed)<<timeStampTmp<<endl;
    // if (mClientId == 0)
    // {
        //todo
    //     //cv::imshow("左目校正图像",imLeft);
    //     //cv::imshow("左目原始图像",cv_ptrLeft->image);
    //     //cv::waitKey(0);
    //     mpSkeleton->img_write(timeStampTmp,vImagesRect[0],cv_ptrDepth->image);//这里只传入了左目 和深度的图像
    // }
        
    //cout<<"OPTimeStamp="<<this->OPTimeStamp<<"x="<<this->ObjectPostition_x<<"y="<<this->ObjectPostition_y<<"z="<<this->ObjectPostition_z<<endl;
    mpTracking->GrabImageMultiple(vImagesRect, timeStampTmp,this->OPTimeStamp,this->ObjectPostition);
    //mpTracking->GrabImageMonocular(cv_ptrLeft->image,timeStampTmp,this->OPTimeStamp,this->ObjectPostition);
    cout<<"end to grab image "<<endl;
}

void ClientHandler::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

ClientHandler::kfptr ClientHandler::GetCurrentRefKFfromTracking()
{
    if(mpTracking->mState < 2)
        return nullptr;
    else
        return mpTracking->GetReferenceKF();
}

ClientHandler::bkfptr ClientHandler::GetCurrentRefBKFsfromTracking()
{
    if(mpTracking->mState < 2)
        return nullptr;
    else
        return mpTracking->GetReferenceBKFs();
}


int ClientHandler::GetNumKFsinLoopFinder()
{
    if(mpLoopFinder)
        return mpLoopFinder->GetNumKFsinQueue();
    else
        return -1;
}

int ClientHandler::GetNumKFsinMapMatcher()
{
    if(mpMapMatcher)
        return mpMapMatcher->GetNumKFsinQueue();
    else
        return -1;
}

void ClientHandler::ClearCovGraph(size_t MapId)
{
    mpMapping->ClearCovGraph(MapId);
}

//#ifdef LOGGING
//void ClientHandler::SetLogger(boost::shared_ptr<mylog> pLogger)
//{
//    mpCC->mpLogger = pLogger;
//}
//#endif

} //end ns
