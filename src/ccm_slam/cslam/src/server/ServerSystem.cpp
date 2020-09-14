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

#include <cslam/server/ServerSystem.h>

namespace cslam{

ServerSystem::ServerSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate, const string &strVocFile)
    : mNh(Nh), mNhPrivate(NhPrivate), mMaxClients(4), mpUID(new estd::UniqueIdDispenser())
{
    params::ShowParams();

    mNhPrivate.param("NumOfClients",mNumOfClients,0);

    if(mNumOfClients < 1)
    {
        ROS_ERROR_STREAM("In \" System::System(...)\": Num od clients < 1");
        throw estd::infrastructure_ex();
    }

    //+++++ load vocabulary +++++

    this->LoadVocabulary(strVocFile);

    //+++++ Create KeyFrame Database +++++
    this->InitializeBKFDB();

    //+++++ Create the Map +++++
    this->InitializeBMaps();

    //+++++ Create the Viewer +++++
    this->InitializeViewer();  

    #ifdef LOGGING
    mpLogger.reset(new estd::mylog(30.0,10000000));
    mptLogger.reset(new thread(&estd::mylog::run,mpLogger));
    #endif
}

void ServerSystem::InitializeClients()
{
    //Cannot be called from constructor - shared_from_this() not available

    if(params::stats::mbWriteKFsToFile)
    {
        for(int it=0;it<4;++it)
        {
            std::stringstream ss;
            ss << params::stats::msOutputDir << "KF_GBA_" << it << ".csv";
            this->CleanWriteOutFile(ss.str());
        }
    }

    //Client 0
    //mpClient0.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpKFDB,mpMap0,0,mpUID,eSystemState::SERVER,string(),mpViewer));
    mpClient0.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpBKFDB,mpBMap0,0,mpUID,eSystemState::SERVER,string(),mpViewer));
    #ifdef LOGGING
    mpClient0->InitializeThreads(mpLogger);
    #else
    mpClient0->InitializeThreads();
    #endif

    //Client 1
    if(mNumOfClients > 1)
    {
        //mpClient1.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpKFDB,mpMap1,1,mpUID,eSystemState::SERVER,string(),mpViewer));
        mpClient1.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpBKFDB,mpBMap1,1,mpUID,eSystemState::SERVER,string(),mpViewer));
        #ifdef LOGGING
        mpClient1->InitializeThreads(mpLogger);
        #else
        mpClient1->InitializeThreads();
        #endif
    }

    //Client 2
    if(mNumOfClients > 2)
    {
        //mpClient2.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpKFDB,mpMap2,2,mpUID,eSystemState::SERVER,string(),mpViewer));
        mpClient2.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpBKFDB,mpBMap2,2,mpUID,eSystemState::SERVER,string(),mpViewer));
        #ifdef LOGGING
        mpClient2->InitializeThreads(mpLogger);
        #else
        mpClient2->InitializeThreads();
        #endif
    }

    //Client 3
    if(mNumOfClients > 3)
    {
        //mpClient3.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpKFDB,mpMap3,3,mpUID,eSystemState::SERVER,string(),mpViewer));
        mpClient3.reset(new ClientHandler(mNh,mNhPrivate,mpVoc,mpBKFDB,mpBMap3,3,mpUID,eSystemState::SERVER,string(),mpViewer));
        #ifdef LOGGING
        mpClient3->InitializeThreads(mpLogger);
        #else
        mpClient3->InitializeThreads();
        #endif
    }

    if(mNumOfClients > mMaxClients)
    {
        cout << "\033[1;33m!!! WARN !!!\033[0m Maximum number of clients is " << mMaxClients << endl;
    }

    this->InitializeBMapMatcher();
}

void ServerSystem::LoadVocabulary(const string &strVocFile)
{
    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVoc.reset(new ORBVocabulary());
    bool bVocLoad = mpVoc->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Failed to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;
}

void ServerSystem::InitializeMaps()
{
    mpMap0.reset(new Map(mNh,mNhPrivate,0,eSystemState::SERVER));
    if(mNumOfClients > 1) mpMap1.reset(new Map(mNh,mNhPrivate,1,eSystemState::SERVER)); else mpMap1=nullptr;
    if(mNumOfClients > 2) mpMap2.reset(new Map(mNh,mNhPrivate,2,eSystemState::SERVER)); else mpMap2=nullptr;
    if(mNumOfClients > 3) mpMap3.reset(new Map(mNh,mNhPrivate,3,eSystemState::SERVER)); else mpMap3=nullptr;
}

void ServerSystem::InitializeBMaps()
{
    mpBMap0.reset(new BundledMap(mNh,mNhPrivate,0,eSystemState::SERVER));
    if(mNumOfClients > 1) mpBMap1.reset(new BundledMap(mNh,mNhPrivate,1,eSystemState::SERVER)); else mpBMap1=nullptr;
    if(mNumOfClients > 2) mpBMap2.reset(new BundledMap(mNh,mNhPrivate,2,eSystemState::SERVER)); else mpBMap2=nullptr;
    if(mNumOfClients > 3) mpBMap3.reset(new BundledMap(mNh,mNhPrivate,3,eSystemState::SERVER)); else mpBMap3=nullptr;
}

void ServerSystem::InitializeKFDB()
{
    mpKFDB.reset(new KeyFrameDatabase(mpVoc));
}

void ServerSystem::InitializeBKFDB()
{
    mpBKFDB.reset(new BundledKeyFramesDatabase(mpVoc));
}

void ServerSystem::InitializeMapMatcher()
{
    mpMapMatcher.reset(new MapMatcher(mNh,mNhPrivate,mpBKFDB,mpVoc,mpBMap0,mpBMap1,mpBMap2,mpBMap3));
    mptMapMatching.reset(new thread(&MapMatcher::Run,mpMapMatcher));
	if(mpClient0)
    {
         commptr comm0 = mpClient0->GetCommPtr();
         mpMapMatcher->SetCommunicator(comm0);//zmf add
    } 
    if(mpClient1)
    {
         commptr comm1 = mpClient1->GetCommPtr();
         mpMapMatcher->SetCommunicator1(comm1);//zmf add
         //mpMapMatcher->SetCommunicator(comm0,comm1);//zmf add
    } 
    if(mpClient2)
    {
         commptr comm2 = mpClient2->GetCommPtr();
         //mpMapMatcher->SetCommunicator(comm0,comm1);//zmf add
    } 
    if(mpClient3)
    {
         commptr comm3 = mpClient3->GetCommPtr();
         //mpMapMatcher->SetCommunicator(comm0,comm1);//zmf add
    } 

    if(mpClient0) mpClient0->SetMapMatcher(mpMapMatcher);
    if(mpClient1) mpClient1->SetMapMatcher(mpMapMatcher);
    if(mpClient2) mpClient2->SetMapMatcher(mpMapMatcher);
    if(mpClient3) mpClient3->SetMapMatcher(mpMapMatcher);
}

void ServerSystem::InitializeBMapMatcher()
{
    mpMapMatcher.reset(new MapMatcher(mNh,mNhPrivate,mpBKFDB,mpVoc,mpBMap0,mpBMap1,mpBMap2,mpBMap3));
    mptMapMatching.reset(new thread(&MapMatcher::Run,mpMapMatcher));
	if(mpClient0)
    {
         commptr comm0 = mpClient0->GetCommPtr();
         mpMapMatcher->SetCommunicator(comm0);//zmf add
    } 
    if(mpClient1)
    {
         commptr comm1 = mpClient1->GetCommPtr();
         mpMapMatcher->SetCommunicator1(comm1);//zmf add
         //mpMapMatcher->SetCommunicator(comm0,comm1);//zmf add
    } 
    if(mpClient2)
    {
         commptr comm2 = mpClient2->GetCommPtr();
         //mpMapMatcher->SetCommunicator(comm0,comm1);//zmf add
    } 
    if(mpClient3)
    {
         commptr comm3 = mpClient3->GetCommPtr();
         //mpMapMatcher->SetCommunicator(comm0,comm1);//zmf add
    } 

    if(mpClient0) mpClient0->SetMapMatcher(mpMapMatcher);
    if(mpClient1) mpClient1->SetMapMatcher(mpMapMatcher);
    if(mpClient2) mpClient2->SetMapMatcher(mpMapMatcher);
    if(mpClient3) mpClient3->SetMapMatcher(mpMapMatcher);
}


void ServerSystem::InitializeViewer()
{
    //create a ccptr
    ccptr pCC{new CentralControl(mNh,mNhPrivate,-1,eSystemState::SERVER,nullptr,nullptr)};

    //create Viewer
    mpViewer.reset(new Viewer(nullptr,pCC));
    mptViewer.reset(new thread(&Viewer::RunServer,mpViewer));
}

void ServerSystem::CleanWriteOutFile(string sFileName)
{
    // Write out the keyframe data
    std::cout << "Cleaning file " << sFileName << std::endl;
    std::ofstream keyframesFile;
    keyframesFile.open(sFileName, std::ofstream::out | std::ofstream::trunc);
    keyframesFile.close();
}

} //end namespace
