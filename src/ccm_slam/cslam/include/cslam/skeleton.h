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


#ifndef CSLAM_SKELETON_H_
#define CSLAM_SKELETON_H_



//C++
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include<vector>
#include <mutex>
//
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
//ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/publisher.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>

//CSLAM
#include <cslam/config.h>
#include <cslam/Tracking.h>

#include <cslam/BundledMap.h>
#include <cslam/Map.h>
#include <cslam/CentralControl.h>

//Msgs
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;
using namespace cv;
using namespace cv::dnn;
using namespace std;

namespace cslam{

//forward decs
//class Tracking;
class Communicator;//zmf add
//----------
struct Skeleton_Frame
{
    double timestamp;//图像时间错
    cv::Mat skeleton_img;//骨架图像
    cv::Mat depth_img;//深度图像

};
class Skeleton : public boost::enable_shared_from_this<Skeleton>
{
public:
    //typedef boost::shared_ptr<Tracking> trackptr;
    typedef boost::shared_ptr<Communicator> commptr; //zmf add
    Net net;

    double timestamp;//图像时间错
    cv::Mat skeleton_img;//骨架图像
    cv::Mat depth_img;//深度图像

    vector<float>skepos;
    std::mutex mMutexNewskeleton;
    std::list<Skeleton_Frame> mlNewSkeletons;
    
    
public:
    commptr mpComm;
    Skeleton();//构造函数 zmf add  需要添加一个comm 指针
    ~Skeleton(){
        
    }
    void SetCommunicator(commptr pComm) {mpComm=pComm;}
    //cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp,double &OPtimestamp,vector<vector<float>>&ObjectPostition);
    void img_write(const double & _timestamp,const cv::Mat &im,const cv::Mat &_depth_im);
    int image_segmentation();//图像分割处理函数
    int skeleton_process();//人体骨架识别函数
    void RunClient();
    //vector<Vec3b> readColors();
  
};

} //end namespace

#endif
