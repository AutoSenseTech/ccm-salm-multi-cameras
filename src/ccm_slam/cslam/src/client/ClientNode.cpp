/*
 * @Author: your name
 * @Date: 2019-12-25 16:24:59
 * @LastEditTime: 2019-12-25 16:36:14
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /ccm_slam/cslam/src/client/ClientNode.cpp
 */
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


#include <cslam/client/ClientSystem.h>

#include <cslam/config.h>


// <hxy>: 绑定ros信息需要这三个头文件. 
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cslam/config.h>

#include <cslam/ClientHandler.h> 
using namespace sensor_msgs;
using namespace message_filters;
// </hxy>:
using namespace dnn;
using namespace cv;
std::vector<std::string> classes; 
int main(int argc, char **argv) {
	cout<<"******************************************************************"<<endl;
    std::cout<<CV_VERSION<<endl;

    

    // <hxy> 此处为了debug方便, 让Node先在开始处停住
    //   char temp[10];
    //   std::cout <<"input anything" << std::endl;
    //   std::cin >> temp;
    //</hxy>

	ros::init(argc, argv, "CSLAM_client_node");

    if(argc != 5)//原本是5个参数 现在又加了一个深度话题名称
    {
        cerr << endl << "Usage: rosrun cslam clientnode path_to_vocabulary path_to_cam_params" << endl;
        ros::shutdown();
        return 1;
    }

    ros::NodeHandle Nh;
    ros::NodeHandle NhPrivate("~");

    // <hxy>
    cslam::ClientSystem myClientSystem(Nh,NhPrivate,argv[1],argv[2]);
//stereo
    // cslam::ClientHandler myClientHandler(myClientSystem.mNh,
    //     myClientSystem.mNhPrivate,myClientSystem.mpVoc,myClientSystem.mpKFDB,
    //     myClientSystem.mpMap,myClientSystem.mClientId,myClientSystem.mpUID,
    //     cslam::eSystemState::CLIENT,myClientSystem.mstrCamFile,nullptr);
//for multi-camera
    cslam::ClientHandler myClientHandlerMulti(myClientSystem.mNh,
        myClientSystem.mNhPrivate,myClientSystem.mpVoc,myClientSystem.mpBKFDB,
        myClientSystem.mpBMap,myClientSystem.mClientId,myClientSystem.mpUID,
        cslam::eSystemState::CLIENT,myClientSystem.mstrCamFile,nullptr);

    //(myClientSystem.mpAgent).reset(&myClientHandler);
    (myClientSystem.mpAgent).reset(&myClientHandlerMulti);
    usleep(10000); //wait to avoid race conditions
    (myClientSystem.mpAgent)->InitializeThreads();
    usleep(10000); //wait to avoid race conditions

    message_filters::Subscriber<sensor_msgs::Image> camera_sub0(myClientHandlerMulti.mNh, argv[3], 1);  // 把topic修改到了launch文件中了，熊亮
    message_filters::Subscriber<sensor_msgs::Image> camera_sub1(myClientHandlerMulti.mNh, argv[4], 1);//这里订阅了三个话题
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(myClientHandlerMulti.mNh, argv[5], 1);
     //for multi camera
    // message_filters::Subscriber<sensor_msgs::Image> camera_sub2(myClientHandlerMulti.mNh, argv[5], 1);
    // message_filters::Subscriber<sensor_msgs::Image> camera_sub3(myClientHandlerMulti.mNh, argv[6], 1);
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(myClientHandlerMulti.mNh, argv[7], 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), camera_sub0,camera_sub1);
    sync.registerCallback(boost::bind(&cslam::ClientHandler::CamImgCb,&myClientHandlerMulti,_1,_2));
    //for multi camera
   // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image,sensor_msgs::Image> sync_pol;
    // message_filters::Synchronizer<sync_pol> sync(sync_pol(10), camera_sub0,camera_sub1,camera_sub2,camera_sub3,depth_sub);
    // sync.registerCallback(boost::bind(&cslam::ClientHandler::CamImgCb,&myClientHandlerMulti,_1,_2,_3,_4,_5));
    // </hxy>://这里需要修改

    ROS_INFO("Started CSLAM client node...");

    ros::Rate r(params::timings::client::miRosRate);

    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
