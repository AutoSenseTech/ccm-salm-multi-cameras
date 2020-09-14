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

#include <cslam/Communicator.h>

namespace cslam {

Communicator::Communicator(ccptr pCC, vocptr pVoc, bmapptr pBMap, bdbptr pBKFDB)
:mpCC(pCC),
      mNh(pCC->mNh), mNhPrivate(pCC->mNhPrivate),
      mpVoc(pVoc), mpBMap(pBMap), mpBundledKeyFramesDatabase(pBKFDB),
      mClientId(pCC->mClientId), mbResetRequested(false),
      mNearestBKfsId(defpair),mpNearestBKFs(nullptr),
      mdPeriodicTime((pCC->mSysState == eSystemState::CLIENT) ? params::comm::client::mfPubPeriodicTime : params::comm::server::mfPubPeriodicTime),
      mdLastTimePub(0.0),mnMaxBKfsIdSent(0),
      mBKfsItBound((pCC->mSysState == eSystemState::CLIENT) ? params::comm::client::miKfItBound : params::comm::server::miKfItBound),
      mMpItBound((pCC->mSysState == eSystemState::CLIENT) ? params::comm::client::miMpItBound : params::comm::server::miMpItBound),
      mBKfsItBoundPub(params::comm::client::miKfPubMax),
      mMpItBoundPub(params::comm::client::miMpPubMax),
      mnEmptyMsgs(0)
{
    mMsgCountLastMapMsg = 0;
    mOutMapCount = 0;
    mServerMapCount = 0;

    mnWeakAckBKFs = KFRANGE;
    mnWeakAckMP = MPRANGE;

    //Topics
    std::stringstream* ss;
    string PubMapTopicName, MapInTopicName, SysType;

    if(mpCC->mSysState == eSystemState::CLIENT)
    {
        SysType = "Client";

        //Subscriber
        mNhPrivate.param("MapInTopicName",MapInTopicName,std::string("nospec"));
        mSubBMap = mNh.subscribe<ccmslam_msgs::BMap>(MapInTopicName,params::comm::client::miSubMapBufferSize,boost::bind(&Communicator::MapCbClient,this,_1));

        //Publisher
        ss = new stringstream;
        *ss << "BMapOut" << SysType << mClientId;
        PubMapTopicName = ss->str();
        mPubBMap = mNh.advertise<ccmslam_msgs::BMap>(PubMapTopicName,params::comm::client::miPubMapBufferSize);

        //todo
		// //client  对骨架线程传给comm的数据进行发布  然后由server端的comm  线程进行接收 写入到comm 的成员变量中
        // if(mClientId==0)
        // {
        //     mPubSke = mNh.advertise<ccmslam_msgs::ObjectPosition>("ObjectPositionTopicName_client",10);
        // }
    }
    else if(mpCC->mSysState == eSystemState::SERVER)
    {
        SysType = "Server";

        //Subscriber
        ss = new stringstream;
        *ss << "MapInTopicName" << mClientId;
        mNhPrivate.param(ss->str(),MapInTopicName,std::string("nospec"));
        mSubBMap = mNh.subscribe<ccmslam_msgs::BMap>(MapInTopicName,params::comm::server::miSubMapBufferSize,boost::bind(&Communicator::MapCbServer,this,_1));
        
		//mSubSke = mNh.subscribe<ccmslam_msgs::ObjectPosition> ("ObjectPositionTopicName_client",10,boost::bind(&Communicator::serversave_skelepos,this,_1));//zmf add
        //Publisher
        ss = new stringstream;
        *ss << "BMapOut" << SysType << mClientId;
        PubMapTopicName = ss->str();
        mPubBMap = mNh.advertise<ccmslam_msgs::BMap>(PubMapTopicName,params::comm::server::miPubMapBufferSize);
		//Publisher
        //server发布关键物体位置信息
        //mPubSke = mNh.advertise<ccmslam_msgs::ObjectPosition>("ObjectPositionTopicName",10);//todo
    }
    else
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ResetIfRequested(): invalid systems state: " << mpCC->mSysState << endl;
        throw infrastructure_ex();
    }

    delete ss;

    if(MapInTopicName=="nospec" )
    {
        cout << "Client " << mClientId << " \033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::Communicator(...): bad IN topic name" << endl;
        throw estd::infrastructure_ex();
    }
}

void Communicator::serversave_skelepos(ccmslam_msgs::ObjectPositionConstPtr pMsg)
{
    //double=pMsg->Timestamp;
    //cout<<"++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<endl;
    //cout<<"server 端comm线程输出骨架信息..."<<endl;
    //cout<<"+++++++timeStamp from serversave_skelepos: "<<setw(20)<<setprecision(15)<<setiosflags(ios::fixed)<<pMsg->Timestamp<<endl;
    //skelePos_list.insert(pair<double, vector<float>>(skele->timestamp, skele->skepos));//在这里插入了变量
    skelePos_list_server[pMsg->Timestamp]=pMsg->Position;
    
    
}
void Communicator::PublishObjectPositionClient()
{
    //1.server接受终端A传过来的世界坐标系下的坐标
    //2.完成A世界坐标系到B世界坐标系下的转换
    //3.ros publich 到B终端

    //server发布server获取到的人体骨架坐标 一共27个三维坐标
    //cout<<"++++++++++++++++++++"<<endl;
    ccmslam_msgs::ObjectPosition mObjectPosition;
    //获取关键物体的位置
    mObjectPosition.Client_ID="A";
    if(skelePos_list.empty())
        return;
    cout<<"----------skelePos_list size before: "<<skelePos_list.size()<<endl;
	auto iter= skelePos_list.begin(); //返回最后一个元素
    mObjectPosition.Timestamp=iter->first;
    //cout<<"+++++++timeStamp PublishObjectPositionClient: "<<setw(20)<<setprecision(15)<<setiosflags(ios::fixed)<<mObjectPosition.Timestamp<<endl;
    auto skeposvec=iter->second;
    cout<<"----------- PublishObjectPositionClient pose size: "<<skeposvec.size()<<endl;
    skelePos_list.erase(iter);
    cout<<"----------skelePos_list size after: "<<skelePos_list.size()<<endl;
    mObjectPosition.Position=skeposvec;
    //cout<<"============"<<endl;
    if(skeposvec.size()>0)
        mPubSke.publish(mObjectPosition);

}

void Communicator::PublishObjectPositionServer()
{
    //1.server接受终端A传过来的世界坐标系下的坐标
    //2.完成A世界坐标系到B世界坐标系下的转换
    //3.ros publich 到B终端

    //server发布server获取到的人体骨架坐标 一共27个三维坐标
    
    if(skelePos_list_receive_from_mapmatch.empty())
        return;
    ccmslam_msgs::ObjectPosition mObjectPosition;
    //获取关键物体的位置
    auto iter= skelePos_list_receive_from_mapmatch.rbegin(); //返回最后一个元素
    mObjectPosition.Client_ID="A";
    mObjectPosition.Timestamp=iter->first;
    mObjectPosition.Position=iter->second;
    mPubSke.publish(mObjectPosition);

}
map<double, vector<float>> Communicator::GetSkelepos()
{
    //add mutex here
    //assert(inputPose.size()==0);
    //mmSkelepos.clear();
    //mmSkelepos = inputPos;
    
    unique_lock<mutex> lock(mMutexBuffersIn);
    return skelePos_list_server;
}
void Communicator::RunClient()
{
    while(true)
    {
        if(params::sys::mbStrictLock)
        {
            while(!mpCC->LockMapping()){
                usleep(params::timings::miLockSleep);
            }
            while(!mpCC->LockTracking()){
                usleep(params::timings::miLockSleep);
            }
        }
        else
        {
            while(!mpBMap->LockBMapUpdate()){usleep(params::timings::miLockSleep);}
        }

        this->PublishMapClient();
		//this->PublishObjectPositionClient(); //todo uncomment

        {
            unique_lock<mutex> lock(mMutexBuffersIn);

            if(!mlBufBKFsin.empty())
            {
                //this->ProcessKfInClient();
                this->ProcessBKfsInClient();
            }

            if(!mlBufMPin.empty())
            {
                this->ProcessMpInClient();
            }
        }

        for(list<bkfptr>::iterator lit = mlpAddedBKfs.begin();lit != mlpAddedBKfs.end();++lit)
        {
            bkfptr pBKFi = *lit;

            if(pBKFi->GetMapPoints().size() == 0)
            {
                pBKFi->SetBadFlag(false,true);
            }
            else
            {
                pBKFi->UpdateConnections();
            }
        }
        mlpAddedBKfs.clear();

        if(params::sys::mbStrictLock)
        {
            mpCC->UnLockMapping();
            mpCC->UnLockTracking();
        }
        else
        {
            mpBMap->UnLockBMapUpdate();
        }

        ResetIfRequested();

        usleep(params::timings::client::miCommRate);
    }
}

void Communicator::RunServer()
{
    while(true)
    {
        #ifdef LOGGING
        mpCC->mpLogger->SetComm(__LINE__,mClientId);
        #endif

        while(!mpCC->LockComm()){usleep(params::timings::miLockSleep);}

        #ifdef LOGGING
        mpCC->mpLogger->SetComm(__LINE__,mClientId);
        #endif

        while(!mpBMap->LockBMapUpdate()){usleep(params::timings::miLockSleep);}

        #ifdef LOGGING
        mpCC->mpLogger->SetComm(__LINE__,mClientId);
        #endif
        this->PublishMapServer();
        //cout<<"(RunServer) publish succ"<<endl;
		//this->PublishObjectPositionServer();

        {
            unique_lock<mutex> lock(mMutexBuffersIn);
           // cout<<"(RunServer) succ"<<endl;
            if(!mlBufBKFsin.empty())
            {
                //cout<<"(RunServer) ProcessBKfsInServer enter"<<endl;
                this->ProcessBKfsInServer();
                //cout<<"(RunServer) ProcessBKfsInServer succ"<<endl;
            }

            if(!mlBufMPin.empty())
            {
                #ifdef TRACELOCK
                mpCC->mpLogger->SetComm(__LINE__,mClientId);
                #endif
                //cout <<"Comm server处理MP消息 ProcessMpInServer" <<endl;
                this->ProcessMpInServer();
                //cout <<"Comm server处理MP消息结束 " <<endl;

                #ifdef TRACELOCK
                mpCC->mpLogger->SetComm(__LINE__,mClientId);
                #endif
            }
        }

        #ifdef TRACELOCK
        mpCC->mpLogger->SetComm(__LINE__,mClientId);
        #endif
      
        mpCC->UnLockComm();
         
        mpBMap->UnLockBMapUpdate();
       

        #ifdef LOGGING
        mpCC->mpLogger->SetComm(__LINE__,mClientId);
        #endif
  
        ResetIfRequested();
    
        usleep(params::timings::server::miCommRate);
 
    }
}

void Communicator::MapCbClient(ccmslam_msgs::BMapConstPtr pMsg)
{
    cout << "线程1:进入MapCbClient" <<endl;
    for(int it = 0; it < pMsg->vAckBKFs.size() ; ++it)
    {
        if(mlBKfsOpenAcks.empty())
        {
            break;
        }

        int IDi = pMsg->vAckBKFs[it];
        list<AckPairBKF>::iterator lit = mlBKfsOpenAcks.begin();
        while(lit != mlBKfsOpenAcks.end())
        {
            AckPairBKF APi = *lit;

            if(APi.first == IDi)
            {
                bkfptr pBKFsi = APi.second;
                pBKFsi->Ack();
                lit = mlBKfsOpenAcks.erase(lit);
                break;
            }
            else if(APi.first < IDi)
            {
                bkfptr pBKFsi = APi.second;
                pBKFsi->SetSendFull();
                lit = mlBKfsOpenAcks.erase(lit);
                continue;
            }

            ++lit;
        }
    }

    //Weak Acks
    size_t nWeakAckBKF = pMsg->WeakAckBKF;

    if(nWeakAckBKF != KFRANGE)
    {
        list<AckPairBKF>::iterator lit = mlBKfsOpenAcks.begin();
        while(lit != mlBKfsOpenAcks.end())
        {
            AckPairBKF APi = *lit;

            if(APi.first <= nWeakAckBKF)
            {
                bkfptr pBKFsi = APi.second;
                pBKFsi->SetSendFull();
                lit = mlBKfsOpenAcks.erase(lit);
                continue;
            }

            ++lit;
        }
    }

    for(int it = 0; it < pMsg->vAckMPs.size() ; ++it)
    {
        if(mlMpOpenAcks.empty())
        {
            break;
        }

        int IDi = pMsg->vAckMPs[it];
        list<AckPairMP>::iterator lit = mlMpOpenAcks.begin();
        while(lit != mlMpOpenAcks.end())
        {
            AckPairMP APi = *lit;

            if(APi.first == IDi)
            {
                mpptr pMPi = APi.second;
                pMPi->Ack();
                lit = mlMpOpenAcks.erase(lit);
                break;
            }
            if(APi.first < IDi)
            {
                mpptr pMPi = APi.second;
                pMPi->SetSendFull();
                lit = mlMpOpenAcks.erase(lit);
                continue;
            }

            ++lit;
        }
    }

    size_t nWeakAckMP = pMsg->WeakAckMP;

    if(nWeakAckMP != MPRANGE)
    {
        list<AckPairMP>::iterator lit = mlMpOpenAcks.begin();
        while(lit != mlMpOpenAcks.end())
        {
            AckPairMP APi = *lit;

            if(APi.first <= nWeakAckMP)
            {
                mpptr pMPi = APi.second;
                pMPi->SetSendFull();
                lit = mlMpOpenAcks.erase(lit);
                continue;
            }

            ++lit;
        }
    }

    //Pack'em in the input buffers

    if(!pMsg->BundledKeyframes.empty())
    {
        if(pMsg->MapPoints.empty())
        {
            return;
        }
        //cout << "Server 有效消息传入 Client" << endl;
        unique_lock<mutex> lock(mMutexBuffersIn);

        ccmslam_msgs::BKF msg = pMsg->BundledKeyframes[0];
        bkfptr pRefBKf = mpBMap->GetBKfsPtr(msg.mpPred_BKfId,msg.mpPred_BKfClientId);
        if(!pRefBKf)
        {
            return;
        }

        mlBufBKFsin.clear(); //only use most recent information
        mlBufMPin.clear();

        //BundledKeyframes
        for(int idx=0;idx<pMsg->BundledKeyframes.size();++idx)
        {
            ccmslam_msgs::BKF msgFull = pMsg->BundledKeyframes[idx];
            ccmslam_msgs::BKFred msgRed;
            msgRed.mClientId = MAPRANGE;
            mlBufBKFsin.push_back(make_pair(msgFull,msgRed));
        }

        //MapPoints
        for(int idx=0;idx<pMsg->MapPoints.size();++idx)
        {
            ccmslam_msgs::MP msgFull = pMsg->MapPoints[idx];
            ccmslam_msgs::MPred msgRed;
            msgRed.mClientId = MAPRANGE;
            mlBufMPin.push_back(make_pair(msgFull,msgRed));
        }
    }
    else
    {
       //cout << "Server 没有　有效消息传入 Client" << endl;
    }
}

void Communicator::MapCbServer(ccmslam_msgs::BMapConstPtr pMsg)
{
    //cout << "++++++++++++++++++MapCbServer++++++++++++++++++++++" << endl;
    {
        unique_lock<mutex> lock(mMutexBuffersIn);

        if(pMsg->BKFUpdates.size() > 0)
        {
            //cout << "+++++ server 更新KF　+++++" << endl;
            for(int idx=0;idx<pMsg->BKFUpdates.size();++idx)
            {
                ccmslam_msgs::BKF msgFull;
                msgFull.mClientId = MAPRANGE;
                ccmslam_msgs::BKFred msgRed = pMsg->BKFUpdates[idx];
                mlBufBKFsin.push_back(make_pair(msgFull,msgRed));
            }
        }
       // cout << "MapCbServer 1" << endl;
        if(pMsg->BundledKeyframes.size() > 0)
        {
            //cout << "+++++ server 添加BKF　+++++" << endl;
            for(int idx=0;idx<pMsg->BundledKeyframes.size();++idx)
            {
                ccmslam_msgs::BKF msgFull = pMsg->BundledKeyframes[idx];
                ccmslam_msgs::BKFred msgRed;
                msgRed.mClientId = MAPRANGE;
                mlBufBKFsin.push_back(make_pair(msgFull,msgRed));
            }
        }
        //cout << "MapCbServer 2" << endl;
        if(pMsg->MPUpdates.size() > 0)
        {
            //cout << "+++++ server 更新MP　+++++" << endl;
            for(int idx=0;idx<pMsg->MPUpdates.size();++idx)
            {
                ccmslam_msgs::MP msgFull;
                msgFull.mClientId = MAPRANGE;
                ccmslam_msgs::MPred msgRed = pMsg->MPUpdates[idx];
                mlBufMPin.push_back(make_pair(msgFull,msgRed));
            }
        }
        //cout << "MapCbServer 3" << endl;
        if(pMsg->MapPoints.size() > 0)
        {
            //cout << "MapCbServer 3.1" << endl;
            for(int idx=0;idx<pMsg->MapPoints.size();++idx)
            {
                //cout << "MapCbServer 3.2" << endl;
                ccmslam_msgs::MP msgFull = pMsg->MapPoints[idx];
                //cout << "MapCbServer 3.3" << endl;
                ccmslam_msgs::MPred msgRed;
                //cout << "MapCbServer 3.4" << endl;
                msgRed.mClientId = MAPRANGE;
                //cout << "MapCbServer 3.5" << endl;
                mlBufMPin.push_back(make_pair(msgFull,msgRed));
                //cout << "MapCbServer 3.6" << endl;
            }
        }
    }
   
    {
        unique_lock<mutex> lock(mMutexNearestBKf);

        mNearestBKfsId = make_pair(pMsg->ClosestBKf_Id,pMsg->ClosestBKf_ClientId);
        if(mNearestBKfsId.first != KFRANGE)
        {
            bkfptr pBKF = mpBMap->GetBKfsPtr(pMsg->ClosestBKf_Id,pMsg->ClosestBKf_ClientId);
            if(pBKF)
                mpNearestBKFs = pBKF;
        }
    }

    #ifdef INTERRUPTBA
    if(pMsg->BKFUpdates.size() > 0 || pMsg->BundledKeyframes.size() > 0 || pMsg->MPUpdates.size() || pMsg->MapPoints.size() > 0)
    {
        if(mpBMap->isRunningGBA())
        {
            mpBMap->StopGBA();
          
        }

        mnEmptyMsgs = 0;
    }
    else
    {
        if(mpBMap->GetMaxBKFid() > 10)
        {
            //make sure map was initialized
          
            ++mnEmptyMsgs;

            const size_t nThresFinished = (size_t)(5.0 * params::comm::client::mfPubFreq);
            //idea: if for n sec only empty msgs arrive, this client should be finished. With working connection, in n sec arrive n*Pub_freq_client msgs
            if(mnEmptyMsgs >= nThresFinished)
            {
                #ifdef LOGGING
                mpCC->mpLogger->SetFinished(mpCC->mClientId);
                #endif

                #ifdef FINALBA
                if(mpBMap->isGBAinterrupted() && !mpBMap->isRunningGBA())
                {
                    cout << "Comm " << mpCC->mClientId << ": Trigger GBA" << endl;
                    mpBMap->RequestBA(mpCC->mClientId);
                    mnEmptyMsgs = 0; //reset this value, otherwise a finished module will request BA in every interation -- not good if other module is still running
                }
                #endif
            }
        }
    }
    #endif
}

void Communicator::PublishMapClient()
{
    double dTimeDiff = ros::Time::now().toSec() - mdLastTimePub;
    if(dTimeDiff < mdPeriodicTime)
        return;

    mdLastTimePub = ros::Time::now().toSec();

    if(mpBMap->BundledKeyFramesInMap()<=params::tracking::miInitKFs) //starting sending not before tracking is initialized stably
        return;

    unique_lock<mutex> lockOut(mMutexBuffersOut);

    ccmslam_msgs::BMap msgBMap;
    bkfptr pBKFsFront;

    int ItCount = 0;

    //BundledKeyframes
    {
        bkfptr pCurBKf;
        set<bkfptr>::iterator sit = mspBufferBKfsOut.begin();

        while(!mspBufferBKfsOut.empty() && (ItCount < mKfItBoundPub)) //Do not call checkBuffer()-method -- it also need the BufferOutMutex
        {
            if(sit == mspBufferBKfsOut.end())
                break;

            pCurBKf = *sit;

            if(pCurBKf->isBad())
            {
                sit = mspBufferBKfsOut.erase(sit);
                continue;
            }

            int nFullBKFs = msgBMap.BundledKeyframes.size(); //we need this to determine whether a full KF was added or not

            pCurBKf->ConvertToMessage(msgBMap);

            pCurBKf->UnMarkInOutBuffer();

            if(msgBMap.BundledKeyframes.size() > nFullBKFs)
            {
                AckPairBKF pAck = make_pair(pCurBKf->mId.first,pCurBKf);
                mlBKfsOpenAcks.push_back(pAck);

                if(pCurBKf->mId.first > mnMaxBKfsIdSent && pCurBKf->mId.second == mClientId)
                    mnMaxBKfsIdSent = pCurBKf->mId.first;
            }

            if(!pBKFsFront)
                pBKFsFront = pCurBKf;

            sit = mspBufferBKfsOut.erase(sit);
            ++ItCount;

            #ifndef HIDEBUFFERLIMITS
            if(ItCount == mKfItBoundPub)
            {
                cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " reached OUTPUT KF iteration limit [buffsize: " << mspBufferKfOut.size() << "]" << endl;
            }
            #endif
        }
    }

    if(!pBKFsFront)
    {
        pBKFsFront = mpBKFsLastFront;
    }

    if(!pBKFsFront && mpCC->mpCH->GetTrackPtr()->mState==Tracking::eTrackingState::OK)
    {
        return;
    }
    else
        mpBKFsLastFront = pBKFsFront;

    ItCount = 0;

    //MapPoints
    {
        mpptr pCurMp;
        set<mpptr>::iterator sit = mspBufferMpOut.begin();

        while(!mspBufferMpOut.empty() && (ItCount < mMpItBoundPub)) //Do not call checkBuffer()-method -- it also need the BufferOutMutex
        {
            if(sit == mspBufferMpOut.end())
                break;

            pCurMp = *sit;

            if(pCurMp->isBad())
            {
                sit = mspBufferMpOut.erase(sit);
                continue;
            }

            if(!pCurMp->IsSent() && mnMaxBKfsIdSent < pCurMp->GetMaxObsBKFsId())
            {
                ++sit;
                continue;
            }

            int nFullMPs = msgBMap.MapPoints.size(); //we need this to determine whether a full MP was added or not

            pCurMp->ConvertToMessage(msgBMap,pBKFsFront);

            pCurMp->UnMarkInOutBuffer();

            if(msgBMap.MapPoints.size() > nFullMPs)
            {
                AckPairMP pAck = make_pair(pCurMp->mId.first,pCurMp);
                mlMpOpenAcks.push_back(pAck);
            }

            sit = mspBufferMpOut.erase(sit);
            ++ItCount;

            #ifndef HIDEBUFFERLIMITS
            if(ItCount == mMpItBoundPub)
            {
                cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " reached OUTPUT MP iteration limit -- size: " << mspBufferMpOut.size() << endl;
            }
            #endif
        }
    }

    bkfptr pClosestBKFs = mpCC->mpCH->GetCurrentRefBKFsfromTracking();

    if(pClosestBKFs)
    {
        msgBMap.ClosestBKf_Id = static_cast<uint16_t>(pClosestBKFs->mId.first);
        msgBMap.ClosestBKf_ClientId = static_cast<uint8_t>(pClosestBKFs->mId.second);
    }
    else
    {
        msgBMap.ClosestBKf_Id = KFRANGE;
        msgBMap.ClosestBKf_ClientId = MAPRANGE;
    }

    bool bSend = !msgBMap.BundledKeyframes.empty() || !msgBMap.BKFUpdates.empty() || !msgBMap.MapPoints.empty() || msgBMap.MPUpdates.empty() || msgBMap.ClosestBKf_Id != KFRANGE;

    if(bSend)
    {
        ++mOutMapCount;
        msgBMap.mMsgId = mOutMapCount;
        msgBMap.header.stamp = ros::Time::now();
        
        mPubBMap.publish(msgBMap);
        cout << "===========================Client 发送消息"<<endl;
    }
}

void Communicator::PublishMapServer()
{
    double dTimeDiff = ros::Time::now().toSec() - mdLastTimePub;
    if(dTimeDiff < mdPeriodicTime)
        return;

    ccmslam_msgs::BMap msgBMap;

    mdLastTimePub = ros::Time::now().toSec();

    //BKF Acks
    for(set<size_t>::iterator sit = msAcksBKF.begin();sit != msAcksBKF.end();)
    {
        size_t id = *sit;
        msgBMap.vAckBKFs.push_back((uint16_t)id);
        sit = msAcksBKF.erase(sit);
    }

    //MP Acks
    for(set<size_t>::iterator sit = msAcksMP.begin();sit != msAcksMP.end();)
    {
        size_t id = *sit;
        msgBMap.vAckMPs.push_back((uint32_t)id);
        sit = msAcksMP.erase(sit);
    }

    //Weak Acks
    msgBMap.WeakAckBKF = (uint16_t)mnWeakAckBKFs;
    mnWeakAckBKFs = KFRANGE;

    msgBMap.WeakAckMP = (uint32_t)mnWeakAckMP;
    mnWeakAckMP = MPRANGE;

    //fill with vicitity information
    if(mpBMap->BundledKeyFramesInMap() > 10) //start after 10 KFs
    {
        unique_lock<mutex> lock(mMutexNearestBKf);

        if(!mpNearestBKFs)
        {
            //start a new attempt to get the pointer
            bkfptr pBKF = mpBMap->GetBKfsPtr(mNearestBKfsId);
            if(pBKF)
                mpNearestBKFs = pBKF;
        }

        if(mpNearestBKFs && mNearestBKfsId.first != KFRANGE)
        {
            if(mpNearestBKFs->mId != mNearestBKfsId)
            {
                //try again to get ptr
                bkfptr pBKF = mpBMap->GetBKfsPtr(mNearestBKfsId);
                if(pBKF)
                    mpNearestBKFs = pBKF;
                else
                {
                    //try the closest ones -- maybe this can be found
                    for(size_t itclose = 1; itclose < 1+SERVERCURKFSEARCHITS; ++itclose)
                    {
                        pBKF = mpBMap->GetBKfsPtr(mNearestBKfsId.first - itclose,mNearestBKfsId.second);
                        if(pBKF)
                        {
                            mpNearestBKFs = pBKF;
                            break;
                        }
                    }
                }
            }

            //if we cannot find the current nearest KF, we use the last valid one
            mpBMap->PackVicinityToMsg(mpNearestBKFs,msgBMap,mpCC);
        }
    }

    //publish (if not empty)

    if(!msgBMap.vAckBKFs.empty() || !msgBMap.vAckMPs.empty() || !msgBMap.BundledKeyframes.empty() || !msgBMap.MapPoints.empty() || !(msgBMap.WeakAckBKF == KFRANGE) || !(msgBMap.WeakAckMP == MPRANGE))
    {
        ++mServerMapCount;
        msgBMap.mMsgId = mServerMapCount;
        msgBMap.header.stamp = ros::Time::now();

        if(params::comm::server::miKfLimitToClient == 0)
        {
            msgBMap.BundledKeyframes.clear();
            msgBMap.MapPoints.clear();
            msgBMap.BKFUpdates.clear();
            msgBMap.MPUpdates.clear();
        }
        mPubBMap.publish(msgBMap);
        // cout <<"Server 发送消息" <<msgMap.ClosestKf_Id<<endl;
    }
}

void Communicator::ProcessKfInClient()
{
    int ItCount = 0;

    while (!mlBufKFin.empty() && (ItCount < mKfItBound))
    {
        msgKFPair msgpair = mlBufKFin.front();
        mlBufKFin.pop_front();

        if(msgpair.first.mClientId != MAPRANGE)
        {
            ccmslam_msgs::KF* pMsg = new ccmslam_msgs::KF();

            *pMsg = msgpair.first;

            {
                idpair RefID = make_pair(pMsg->mpPred_KfId,pMsg->mpPred_KfClientId);
                kfptr pRefKf = mpMap->GetKfPtr(RefID);
                if(!pRefKf)
                {
                    delete pMsg;
                    continue;
                }
            }
            if(pMsg->mbBad)
            {
                delete pMsg;
                continue;
            }

            kfptr pKF = mpMap->GetKfPtr(pMsg->mnId,pMsg->mClientId);

            if(pKF)
            {
                if(pMsg->mbPoseChanged)
                {

                    pKF->UpdateFromMessage(pMsg);

                    pKF->UpdateConnections();
                }

            }
            else
            {
                pKF.reset(new KeyFrame(pMsg,mpVoc,mpMap,mpDatabase,shared_from_this(),mpCC->mSysState));

                pKF->mdServerTimestamp = ros::Time::now().toNSec();

                if(pKF->isBad())
                {
                    delete pMsg;
                    continue;
                }

                pKF->EstablishInitialConnectionsClient();
                if(pKF->isBad())
                {
                    delete pMsg;
                    continue;
                }

                mpMap->AddKeyFrame(pKF);
                mlpAddedKfs.push_back(pKF);
            }

            delete pMsg;
        }
        else if(msgpair.second.mClientId != MAPRANGE)
        {
            cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " Reception of reduced KF not implemented" << endl;
            throw estd::infrastructure_ex();   
        }

        ++ItCount;

        #ifndef HIDEBUFFERLIMITS
        if(ItCount == mKfItBound)
        {
            cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " -- Agent " << mpCC->mClientId << ": reached INPUT KF iteration limit" << "[" << mlBufKFin.size() << "]" << endl;
        }
        #endif
    }
}

void Communicator::ProcessBKfsInClient()
{
    int ItCount = 0;

    while (!mlBufBKFsin.empty() && (ItCount < mBKfsItBound))
    {
        msgBKFPair msgpair = mlBufBKFsin.front();
        mlBufBKFsin.pop_front();

        if(msgpair.first.mClientId != MAPRANGE)
        {
            ccmslam_msgs::BKF* pMsg = new ccmslam_msgs::BKF();

            *pMsg = msgpair.first;

            {
                idpair RefID = make_pair(pMsg->mpPred_BKfId,pMsg->mpPred_BKfClientId);
                bkfptr pRefBKfs = mpBMap->GetBKfsPtr(RefID);
                if(!pRefBKfs)
                {
                    delete pMsg;
                    continue;
                }
            }
            if(pMsg->mbBad)
            {
                delete pMsg;
                continue;
            }

            bkfptr pBKFs = mpBMap->GetBKfsPtr(pMsg->mnId,pMsg->mClientId);

            if(pBKFs)
            {
                if(pMsg->mbPoseChanged)
                {

                    pBKFs->UpdateFromMessage(pMsg);  

                    pBKFs->UpdateConnections();
                }

            }
            else
            {
                pBKFs.reset(new BundledKeyFrames(pMsg,mpVoc,mpBMap,mpBundledKeyFramesDatabase,shared_from_this(),mpCC->mSysState));

                pBKFs->mdServerTimestamp = ros::Time::now().toNSec();

                if(pBKFs->isBad())
                {
                    delete pMsg;
                    continue;
                }

                pBKFs->EstablishInitialConnectionsClient();
                if(pBKFs->isBad())
                {
                    delete pMsg;
                    continue;
                }

                mpBMap->AddBundledKeyFrames(pBKFs);
                mlpAddedBKfs.push_back(pBKFs);
            }

            delete pMsg;
        }
        else if(msgpair.second.mClientId != MAPRANGE)
        {
            cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " Reception of reduced KF not implemented" << endl;
            throw estd::infrastructure_ex();   
        }

        ++ItCount;

        #ifndef HIDEBUFFERLIMITS
        if(ItCount == mKfItBound)
        {
            cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " -- Agent " << mpCC->mClientId << ": reached INPUT KF iteration limit" << "[" << mlBufKFin.size() << "]" << endl;
        }
        #endif
    }
}

void Communicator::ProcessKfInServer()
{
    int ItCount = 0;

    while (!mlBufKFin.empty() && (ItCount < mKfItBound))
    {
        msgKFPair msgpair = mlBufKFin.front();
        mlBufKFin.pop_front();

        if(msgpair.first.mClientId != MAPRANGE)
        {
            ccmslam_msgs::KF* pMsg = new ccmslam_msgs::KF();

            *pMsg = msgpair.first;
            {
                kfptr pKF = mpMap->GetKfPtr(pMsg->mnId,pMsg->mClientId);
                if(pKF)
                {
                    //Note: this can happen, if the Ack from server to client gets lost
                    msAcksKF.insert(pMsg->mnId);
                    delete pMsg;
                    continue;
                }
            }
            if(mpMap->IsKfDeleted(pMsg->mnId,pMsg->mClientId))
            {
                //Note: this can happen, if the Ack from server to client gets lost
                msAcksKF.insert(pMsg->mnId);
                delete pMsg;
                continue;
            }

            kfptr pKF{new KeyFrame(pMsg,mpVoc,mpMap,mpDatabase,shared_from_this(),mpCC->mSysState,mpCC->mpUID->GetId(),mpCC->mg2oS_wcurmap_wclientmap)};
            pKF->mdServerTimestamp = ros::Time::now().toNSec();

            if(pKF->isBad())
            {
                //could not be processed, but send weak ack
                this->SetWeakAckKF(pMsg->mnId);
                delete pMsg;
                continue;
            }

            pKF->EstablishInitialConnectionsServer();
            if(pKF->isBad())
            {
                this->SetWeakAckKF(pMsg->mnId);
                delete pMsg;
                continue;
            }

            pKF->UpdateConnections();

            #ifdef DEBUGGING2
            {
                kfptr pKFp = pKF->GetParent();
                if(pKFp)
                {
                    if(pKFp->mId == pKF->mId)
                    {
                        std::cout << COUTERROR << "KF " << pKF->mId.first << "|" << pKF->mId.second << " : is its own parent" << std::endl;
                    }
                }
                else
                {
                    if(!(pKF->mId.first==0))
                        std::cout << COUTERROR << "KF " << pKF->mId.first << "|" << pKF->mId.second << " : no parent" << std::endl;
                }
            }
            #endif

            mpMap->AddKeyFrame(pKF);
            mpMapping->InsertKeyFrame(pKF);
            msAcksKF.insert(pMsg->mnId);

            if(pKF->mId.first == 0)
            {
                mpMap->mvpKeyFrameOrigins.push_back(pKF);
            }

            delete pMsg;
        }
        else if(msgpair.second.mClientId != MAPRANGE)
        {
            ccmslam_msgs::KFred* pMsg = new ccmslam_msgs::KFred();

            *pMsg = msgpair.second;

            kfptr pKF = mpMap->GetKfPtr(pMsg->mnId,pMsg->mClientId);

            if(pKF)
            {
                //everything ok
            }
            else
            {
                if(!mpMap->IsKfDeleted(pMsg->mnId,pMsg->mClientId))
                {
                    this->SetWeakAckKF(pMsg->mnId);
                }
            }

            if(!pKF)
            {
                delete pMsg;
                continue; //maybe it is deleted, maybe it got lost -- but it is not there
            }

            if(pKF->isBad())
            {
                delete pMsg;
                continue; //no need to process bad KFs
            }

            pKF->UpdateFromMessage(pMsg,mpCC->mg2oS_wcurmap_wclientmap);
            pKF->UpdateConnections();

            delete pMsg;
        }

        ++ItCount;

        #ifndef HIDEBUFFERLIMITS
        if(ItCount == mKfItBound)
        {
            cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " -- Agent " << mpCC->mClientId << ": reached INPUT KF iteration limit" << "[" << mlBufKFin.size() << "]" << endl;
        }
        #endif
    }
}

void Communicator::ProcessBKfsInServer()
{
    int ItCount = 0;

    while (!mlBufBKFsin.empty() && (ItCount < mBKfsItBound))
    {
        msgBKFPair msgpair = mlBufBKFsin.front();
        mlBufBKFsin.pop_front();

        if(msgpair.first.mClientId != MAPRANGE)  //new
        {
            ccmslam_msgs::BKF* pMsg = new ccmslam_msgs::BKF();

            *pMsg = msgpair.first;
            {
                //这一帧在server的地图里面已经有了 update的bkf
                bkfptr pBKF = mpBMap->GetBKfsPtr(pMsg->mnId,pMsg->mClientId);
                if(pBKF)
                {
                    //Note: this can happen, if the Ack from server to client gets lost
                    msAcksBKF.insert(pMsg->mnId);
                    delete pMsg;
                    continue;
                }
            }
            if(mpBMap->IsBKfDeleted(pMsg->mnId,pMsg->mClientId))
            {
                //Note: this can happen, if the Ack from server to client gets lost
                msAcksBKF.insert(pMsg->mnId);
                delete pMsg;
                continue;
            }
          
            bkfptr pBKF{new BundledKeyFrames(pMsg,mpVoc,mpBMap,mpBundledKeyFramesDatabase,shared_from_this(),mpCC->mSysState,mpCC->mpUID->GetId(),mpCC->mg2oS_wcurmap_wclientmap)};
            
            pBKF->mdServerTimestamp = ros::Time::now().toNSec();

            if(pBKF->isBad())
            {
                //could not be processed, but send weak ack
                this->SetWeakAckBKF(pMsg->mnId);
                delete pMsg;
                continue;
            }

            pBKF->EstablishInitialConnectionsServer();
            if(pBKF->isBad())
            {
                this->SetWeakAckBKF(pMsg->mnId);
                delete pMsg;
                continue;
            }

            pBKF->UpdateConnections();

            mpBMap->AddBundledKeyFrames(pBKF);
            mpMapping->InsertBundledKeyFrames(pBKF);
            msAcksBKF.insert(pMsg->mnId);

            if(pBKF->mId.first == 0)
            {
                mpBMap->mvpBundledKeyFramesOrigins.push_back(pBKF);
            }

            delete pMsg;
        }
        else if(msgpair.second.mClientId != MAPRANGE)  //update
        {
            ccmslam_msgs::BKFred* pMsg = new ccmslam_msgs::BKFred();

            *pMsg = msgpair.second;

            bkfptr pBKF = mpBMap->GetBKfsPtr(pMsg->mnId,pMsg->mClientId);

            if(pBKF)
            {
                //everything ok
            }
            else
            {
                if(!mpBMap->IsBKfDeleted(pMsg->mnId,pMsg->mClientId))
                {
                    this->SetWeakAckBKF(pMsg->mnId);
                }
            }

            if(!pBKF)
            {
                delete pMsg;
                continue; //maybe it is deleted, maybe it got lost -- but it is not there
            }

            if(pBKF->isBad())
            {
                delete pMsg;
                continue; //no need to process bad KFs
            }

            pBKF->UpdateFromMessage(pMsg,mpCC->mg2oS_wcurmap_wclientmap);
            pBKF->UpdateConnections();

            delete pMsg;
        }

        ++ItCount;

        #ifndef HIDEBUFFERLIMITS
        if(ItCount == mBKfsItBound)
        {
            cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " -- Agent " << mpCC->mClientId << ": reached INPUT KF iteration limit" << "[" << mlBufBKFsin.size() << "]" << endl;
        }
        #endif
    }
}

void Communicator::ProcessMpInClient()
{
    int ItCount = 0;

    while (!mlBufMPin.empty() && (ItCount < mMpItBound))
    {
        msgMPPair msgpair = mlBufMPin.front();
        mlBufMPin.pop_front();

        if(msgpair.first.mClientId != MAPRANGE) // 从server发出的Msg一定是Client第一级相邻关键帧．
        {
            ccmslam_msgs::MP* pMsg = new ccmslam_msgs::MP();

            *pMsg = msgpair.first;

            {
                idpair RefID = make_pair(pMsg->mpPredBKFId,pMsg->mpPredBKFClientId);
                bkfptr pRefBKfs = mpBMap->GetBKfsPtr(RefID);
                if(!pRefBKfs)
                {
                    delete pMsg;
                    continue;
                }
            }

            if(pMsg->mbBad)
            {
                delete pMsg;
                continue;
            }

            mpptr pMP = mpBMap->GetMpPtr(pMsg->mnId,pMsg->mClientId);

            if(pMP)
            {
                if(pMsg->mbPoseChanged)
                {
                    pMP->UpdateFromMessage(pMsg);

                    pMP->UpdateNormalAndDepthPlus();
                }
            }
            else
            {
                pMP.reset(new MapPoint(pMsg,mpBMap,shared_from_this(),mpCC->mSysState));

                if(pMP->isBad())
                {
                    delete pMsg;
                    continue;
                }

                pMP->EstablishInitialConnectionsClient();

                if(pMP->isBad())
                {
                    delete pMsg;
                    continue;
                }

                mpBMap->AddMapPoint(pMP);
            }

            delete pMsg;
        }
        else if(msgpair.second.mClientId != MAPRANGE)
        {
            cout << "\033[1;31m!!!!! FATAL ERROR !!!!!\033[0m " << __func__ << ":"  << __LINE__ << " Reception of reduced KF not implemented" << endl;
            throw infrastructure_ex();
        }

        ++ItCount;
        #ifndef HIDEBUFFERLIMITS
        if(ItCount == mMpItBound)
        {
            cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " -- Agent " << mpCC->mClientId << ": reached INPUT MP iteration limit" << "[" << mlBufMPin.size() << "]" << endl;
        }
        #endif
    }
}

void Communicator::ProcessMpInServer()
{
    int ItCount = 0;

    while (!mlBufMPin.empty() && (ItCount < mMpItBound))
    {
        msgMPPair msgpair = mlBufMPin.front();
        mlBufMPin.pop_front();

        if(msgpair.first.mClientId != MAPRANGE)
        {
            ccmslam_msgs::MP* pMsg = new ccmslam_msgs::MP();

            *pMsg = msgpair.first;

            {
                mpptr pMP = mpBMap->GetMpPtr(pMsg->mnId,pMsg->mClientId);
                if(pMP)
                {
                    //Note: this can happen, if the Ack from server to client gets lost
                    msAcksMP.insert(pMsg->mnId);
                    delete pMsg;
                    continue;
                }
            }
            if(mpBMap->IsMpDeleted(pMsg->mnId,pMsg->mClientId))
            {
                //Note: this can happen, if the Ack from server to client gets lost
                msAcksMP.insert(pMsg->mnId);
                delete pMsg;
                continue;
            }

            mpptr pMP{new MapPoint(pMsg,mpBMap,shared_from_this(),mpCC->mSysState,mpCC->mpUID->GetId(),mpCC->mg2oS_wcurmap_wclientmap)};

            if(pMP->isBad())
            {
                //could not be processed, but send weak ack
                this->SetWeakAckMP(pMsg->mnId);
                delete pMsg;
                continue;
            }

            pMP->EstablishInitialConnectionsServer();

            if(pMP->isBad())
            {
                this->SetWeakAckMP(pMsg->mnId);
                delete pMsg;
                continue;
            }

            mpBMap->AddMapPoint(pMP);
            msAcksMP.insert(pMsg->mnId);

            delete pMsg;
        }
        else if(msgpair.second.mClientId != MAPRANGE) //update
        {
            ccmslam_msgs::MPred* pMsg = new ccmslam_msgs::MPred();

            *pMsg = msgpair.second;

            mpptr pMP = mpBMap->GetMpPtr(pMsg->mnId,pMsg->mClientId);

            if(pMP)
            {
                //everything ok
            }
            else
            {
                if(!mpBMap->IsMpDeleted(pMsg->mnId,pMsg->mClientId))
                {
                    this->SetWeakAckMP(pMsg->mnId);
                }
            }

            if(!pMP)
            {    
                delete pMsg;
                continue; //maybe it is deleted, maybe it got lost -- but it is not there
            }

            if(pMP->isBad())
            {
                delete pMsg;
                continue; //no need to process bad MPs.
            }

            if(pMsg->mbBad)
            {
                delete pMsg;
                continue;
            }

            pMP->UpdateFromMessage(pMsg,mpCC->mg2oS_wcurmap_wclientmap);

            pMP->UpdateNormalAndDepthPlus();

            delete pMsg;
        }

        ++ItCount;

        #ifndef HIDEBUFFERLIMITS
        if(ItCount == mMpItBound)
        {
            cout << "\033[1;34m!!! NOTICE !!!\033[0m " << __func__ << ":" << __LINE__ << " -- Agent " << mpCC->mClientId << ": reached INPUT MP iteration limit" << "[" << mlBufMPin.size() << "]" << endl;
        }
        #endif
    }
}

void Communicator::PassKftoComm(kfptr pKf)
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);

    mspBufferKfOut.insert(pKf);
    pKf->MarkInOutBuffer();
}

void Communicator::PassBKfstoComm(bkfptr pBKfs)
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);

    mspBufferBKfsOut.insert(pBKfs);
    pBKfs->MarkInOutBuffer();
}


void Communicator::PassSkPostoComm(skptr skele) // zmf add 将骨架信息传输到comm  修改comm的成员变量
{
    //这里负责传输骨架信息
    unique_lock<mutex> lockOut(mMutexBuffersOut);

    //skele 为skeleton 的一个share_ptr

    
    //cout<<"client skeleton time: "<<setw(20)<<setprecision(15)<<setiosflags(ios::fixed)<<skele->timestamp<<endl;
    cout<<"----------------client 0 skeletion pose size: "<<skele->skepos.size()<<endl;
    if(skele->skepos.size()>0)
        skelePos_list[skele->timestamp] = skele->skepos;
    
    cout<<"-------------------client 0 skeletion second (pose) size: "<<skelePos_list[skele->timestamp].size()<<endl;
    

    
}

void Communicator::PassMptoComm(mpptr pMp)
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);

    mspBufferMpOut.insert(pMp);
    pMp->MarkInOutBuffer();
}
//typedef boost::shared_ptr<MapMatcher> matchptr;
void Communicator::PassSkePointstoComm(map<double, vector<float>> &result)
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    auto iter=result.rbegin();
    skelePos_list_receive_from_mapmatch[iter->first] = iter->second;
    cout<<"skelePos_list_receive_from_mapmatch have size:" << skelePos_list_receive_from_mapmatch.rbegin()->second.size()<<endl;

}
void Communicator::DeleteMpFromBuffer(mpptr pMP)
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);

    set<mpptr>::iterator sit = find(mspBufferMpOut.begin(),mspBufferMpOut.end(),pMP);
    if(sit != mspBufferMpOut.end())
    {
        mspBufferMpOut.erase(sit);
    }
}

bool Communicator::CheckBufferKfIn()
{
    unique_lock<mutex> lock(mMutexBuffersIn);
    return(!mlBufKFin.empty());
}

bool Communicator::CheckBufferKfOut()
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    return(!mspBufferKfOut.empty());
}

bool Communicator::CheckBufferMpIn()
{
    unique_lock<mutex> lock(mMutexBuffersIn);
    return(!mlBufMPin.empty());
}

bool Communicator::CheckBufferMpOut()
{
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    return(!mspBufferMpOut.empty());
}

void Communicator::RequestReset()
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
            {
                break;
            }
        }
        if(mpCC->mSysState == eSystemState::CLIENT)
            usleep(params::timings::client::miCommRate);
        else if(mpCC->mSysState == eSystemState::SERVER)
            usleep(params::timings::server::miCommRate);
        else KILLSYS
    }
}

void Communicator::ResetIfRequested()
{
    unique_lock<mutex> lockReset(mMutexReset);

    if(mbResetRequested)
    {
        if(mpCC->mSysState == eSystemState::CLIENT)
        {
            this->ResetCommunicator();
        }
        else if(mpCC->mSysState == eSystemState::SERVER)
        {
            this->ResetCommunicator();
            this->ResetMapping();
            this->ResetBundledKeyframeDatabase();
            this->ResetBMap();
        }
        else
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m Communicator::ResetIfRequested(): invalid systems state: " << mpCC->mSysState << endl;
            throw infrastructure_ex();
        }
         mbResetRequested = false;
    }
}

void Communicator::ResetCommunicator()
{
    unique_lock<mutex> lockIn(mMutexBuffersIn);
    unique_lock<mutex> lockOut(mMutexBuffersOut);
    unique_lock<mutex> lock5(mMutexNearestBKf);

    usleep(10000); // wait to give msg buffers time to empty

    mlBufBKFsin.clear();
    mlBufMPin.clear();
    mspBufferBKfsOut.clear();
    mspBufferMpOut.clear();

    msAcksBKF.clear();
    msAcksMP.clear();

    mpNearestBKFs = nullptr;
    mNearestBKfsId = defpair;
}

void Communicator::ResetDatabase()
{
    vector<kfptr> vpKFs = mpMap->GetAllKeyFrames();

    for(vector<kfptr>::iterator vit=vpKFs.begin();vit!=vpKFs.end();++vit)
    {
        kfptr pKF = *vit;
        mpDatabase->erase(pKF);
    }

    mpDatabase->ResetMPs();
}

void Communicator::ResetBundledKeyframeDatabase()
{
    vector<bkfptr> vpBKFs = mpBMap->GetAllBundledKeyFrames();

    for(vector<bkfptr>::iterator vit=vpBKFs.begin();vit!=vpBKFs.end();++vit)
    {
        bkfptr pBKF = *vit;
        mpBundledKeyFramesDatabase->erase(pBKF);
    }

    mpBundledKeyFramesDatabase->ResetMPs();
}

void Communicator::ResetMapping()
{
    mpMapping->RequestReset();
}

void Communicator::ResetMap()
{
    mpMap->clear();
}

void Communicator::ResetBMap()
{
    mpBMap->clear();
}

void Communicator::SetWeakAckKF(size_t id)
{
    if(mnWeakAckKF == KFRANGE)
        mnWeakAckKF = id;
    else if(id > mnWeakAckKF)
        mnWeakAckKF = id;
}

void Communicator::SetWeakAckBKF(size_t id)
{
    if(mnWeakAckBKFs == KFRANGE)
        mnWeakAckBKFs = id;
    else if(id > mnWeakAckBKFs)
        mnWeakAckBKFs = id;
}

void Communicator::SetWeakAckMP(size_t id)
{
    if(mnWeakAckMP == MPRANGE)
        mnWeakAckMP = id;
    else if(id > mnWeakAckMP)
        mnWeakAckMP = id;
}

bool Communicator::kfcmp::operator ()(const kfptr pA, const kfptr pB) const
{
    return pA->mId.first < pB->mId.first;
}

bool Communicator::bkfcmp::operator ()(const bkfptr pA, const bkfptr pB) const
{
    return pA->mId.first < pB->mId.first;
}

bool Communicator::mpcmp::operator ()(const mpptr pA, const mpptr pB) const
{
    return pA->mId.first < pB->mId.first;
}

} //end ns
