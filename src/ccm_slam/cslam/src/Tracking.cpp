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

#include <cslam/Tracking.h>
using namespace std;

namespace cslam {
//todo :cameranum 2->4
// Tracking::Tracking(ccptr pCC, vocptr pVoc, viewptr pFrameViewer, mapptr pMap, dbptr pKFDB, const string &strCamPath, size_t ClientId)//const int sensor
//     : mState(NO_IMAGES_YET),cameraNum(2), mpCC(pCC),mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB), mpInitializer(nullptr),
//       mpViewer(pFrameViewer), mpMap(pMap), mLastRelocFrameId(make_pair(0,0)), mClientId(ClientId), mSensor(1)
// {
//     fout_traj.open( "/home/slam-client/ccmslam_stereo_traj.txt",ios::out|ios::trunc);///////add by zmf  @2019.12.27
//     cv::FileStorage fSettings(strCamPath, cv::FileStorage::READ);
//     float fx = fSettings["Camera.fx"];
//     float fy = fSettings["Camera.fy"];
//     float cx = fSettings["Camera.cx"];
//     float cy = fSettings["Camera.cy"];

//     cv::Mat K = cv::Mat::eye(3,3,CV_32F);
//     K.at<float>(0,0) = fx;
//     K.at<float>(1,1) = fy;
//     K.at<float>(0,2) = cx;
//     K.at<float>(1,2) = cy;
//     K.copyTo(mK);

//     cv::Mat DistCoef(4,1,CV_32F);
//     DistCoef.at<float>(0) = fSettings["Camera.k1"];
//     DistCoef.at<float>(1) = fSettings["Camera.k2"];
//     DistCoef.at<float>(2) = fSettings["Camera.p1"];
//     DistCoef.at<float>(3) = fSettings["Camera.p2"];
//     const float k3 = fSettings["Camera.k3"];
//     if(k3!=0)
//     {
//         DistCoef.resize(5);
//         DistCoef.at<float>(4) = k3;
//     }
//     DistCoef.copyTo(mDistCoef);

//     // 双目摄像头baseline * fx 50
//     mbf = fSettings["Camera.bf"];
//     cout<<"mbf=========================================================================================="<<mbf<<endl;

//     float fps = fSettings["Camera.fps"];
//     if(fps==0)
//         fps=30;

//     // Max/Min Frames to insert keyframes and to check relocalisation
//     mMinFrames = 0;
//     mMaxFrames = fps;

//     cout << endl << "Camera Parameters: " << endl;
//     cout << "- fx: " << fx << endl;
//     cout << "- fy: " << fy << endl;
//     cout << "- cx: " << cx << endl;
//     cout << "- cy: " << cy << endl;
//     cout << "- k1: " << DistCoef.at<float>(0) << endl;
//     cout << "- k2: " << DistCoef.at<float>(1) << endl;
//     if(DistCoef.rows==5)
//         cout << "- k3: " << DistCoef.at<float>(4) << endl;
//     cout << "- p1: " << DistCoef.at<float>(2) << endl;
//     cout << "- p2: " << DistCoef.at<float>(3) << endl;
//     cout << "- fps: " << fps << endl;
    
//     int nRGB = fSettings["Camera.RGB"];
//     mbRGB = nRGB;
//     if(mbRGB)
//         cout << "- color order: RGB (ignored if grayscale)" << endl;
//     else
//         cout << "- color order: BGR (ignored if grayscale)" << endl;    

//     const int nFeatures = params::extractor::miNumFeatures;
//     const float fScaleFactor = params::extractor::mfScaleFactor;
//     const int nLevels = params::extractor::miNumLevels;
//     const int iIniThFAST = params::extractor::miIniThFAST;
//     const int iMinThFAST = params::extractor::miNumThFAST;


//     mpORBextractor.reset(new ORBextractor(nFeatures,fScaleFactor,nLevels,iIniThFAST,iMinThFAST
//                                           ));
//     int sensor = 1;
//     if (sensor == CentralControl::STEREO)  //对传感器类型的判断是否正确
//         mpORBextractorRight.reset(new ORBextractor(nFeatures,fScaleFactor,nLevels,iIniThFAST,iMinThFAST) );

//     if (sensor == CentralControl::MONOCULAR)
//         mpIniORBextractor.reset(new ORBextractor(2*nFeatures,fScaleFactor,nLevels,iIniThFAST,iMinThFAST ));

//     cout << endl  << "ORB Extractor Parameters: " << endl;
//     cout << "- Number of Features: " << nFeatures << endl;
//     cout << "- Scale Levels: " << nLevels << endl;
//     cout << "- Scale Factor: " << fScaleFactor << endl;
//     cout << "- Initial Fast Threshold: " << iIniThFAST << endl;
//     cout << "- Minimum Fast Threshold: " << iMinThFAST << endl;

//     //加入双目需要的远近点门限，记得在配置文件中加入该参数
//     if(sensor == CentralControl::STEREO)
//     {
//         mThDepth = mbf * (float)fSettings["ThDepth"] / fx;
//         cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
//     }

//     //valid the successful pointer
//     if(!mpMap || !mpORBVocabulary || !mpKeyFrameDB || !mpCC)
//     {
//         cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ": nullptr given"<< endl;
//         if(!mpMap) cout << "mpMap == nullptr" << endl;
//         if(!mpORBVocabulary) cout << "mpORBVocabulary == nullptr" << endl;
//         if(!mpKeyFrameDB) cout << "mpKeyFrameDB == nullptr" << endl;
//         if(!mpCC) cout << "mpCC == nullptr" << endl;
//         throw estd::infrastructure_ex();
//     }

//     if(!mpViewer)
//     {
//         cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ": nullptr given"<< endl;
//         if(!mpViewer) cout << "mpViewer == nullptr" << endl;
//         throw estd::infrastructure_ex();
//     }
// }

Tracking::Tracking(ccptr pCC, vocptr pVoc, viewptr pFrameViewer, bmapptr pBMap, bdbptr pBKFDB, const string &strCamPath, size_t ClientId)
:mState(NO_IMAGES_YET),cameraNum(2),mpCC(pCC),mpORBVocabulary(pVoc), mpBundledKeyFramesDB(pBKFDB), mpInitializer(nullptr),
 mpViewer(pFrameViewer), mpBMap(pBMap), mLastRelocFrameId(make_pair(0,0)), mClientId(ClientId), mSensor(3)
{
    fout_traj.open( "/home/slam-client/ccmslam_stereo_traj.txt",ios::out|ios::trunc);///////add by zmf  @2019.12.27
    cv::FileStorage fSettings(strCamPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);
    //multi-camera
    mvK.push_back(mK);
    mvK.push_back(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);
    //multi-camera
    mvDistCoef.push_back(DistCoef);
    mvDistCoef.push_back(DistCoef);  

    // 双目摄像头baseline * fx 50
    mbf = fSettings["Camera.bf"];
    cout<<"mbf=========================================================================================="<<mbf<<endl;

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;
    
    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;
    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;    

    const int nFeatures = params::extractor::miNumFeatures;
    const float fScaleFactor = params::extractor::mfScaleFactor;
    const int nLevels = params::extractor::miNumLevels;
    const int iIniThFAST = params::extractor::miIniThFAST;
    const int iMinThFAST = params::extractor::miNumThFAST;


    mpORBextractor.reset(new ORBextractor(nFeatures,fScaleFactor,nLevels,iIniThFAST,iMinThFAST
                                          ));
   
    if (mSensor == CentralControl::STEREO)  //对传感器类型的判断是否正确
        mpORBextractorRight.reset(new ORBextractor(nFeatures,fScaleFactor,nLevels,iIniThFAST,iMinThFAST) );

    if (mSensor == CentralControl::MONOCULAR)
        mpIniORBextractor.reset(new ORBextractor(2*nFeatures,fScaleFactor,nLevels,iIniThFAST,iMinThFAST ));

    if(mSensor==CentralControl::MULTIPLE)
    {
        mvpORBextractorMultiple.resize(cameraNum);
        for(int i =  0; i < cameraNum; i++)
        {
            mvpORBextractorMultiple[i].reset(new ORBextractor(nFeatures,fScaleFactor,nLevels,iIniThFAST,iMinThFAST));
        }
    }
    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout<< "- Number of Camera: "<< cameraNum <<endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << iIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << iMinThFAST << endl;

    //加入双目需要的远近点门限，记得在配置文件中加入该参数
    if(mSensor == CentralControl::STEREO || mSensor==CentralControl::MULTIPLE)
    {
        mThDepth = mbf * (float)fSettings["ThDepth"] / fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    //valid the successful pointer
    if(!mpBMap || !mpORBVocabulary || !mpBundledKeyFramesDB || !mpCC)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ": nullptr given"<< endl;
        if(!mpBMap) cout << "mpBMap == nullptr" << endl;
        if(!mpORBVocabulary) cout << "mpORBVocabulary == nullptr" << endl;
        if(!mpBundledKeyFramesDB) cout << "mpBundledKeyFramesDB == nullptr" << endl;
        if(!mpCC) cout << "mpCC == nullptr" << endl;
        throw estd::infrastructure_ex();
    }

    if(!mpViewer)
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ": nullptr given"<< endl;
        if(!mpViewer) cout << "mpViewer == nullptr" << endl;
        throw estd::infrastructure_ex();
    }

}

// cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
// {
//     mImGray = im;

//     if(mImGray.channels()==3)
//     {
//         if(mbRGB)
//             cvtColor(mImGray,mImGray,CV_RGB2GRAY);
//         else
//             cvtColor(mImGray,mImGray,CV_BGR2GRAY);
//     }
//     else if(mImGray.channels()==4)
//     {
//         if(mbRGB)
//             cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
//         else
//             cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
//     }

//     //这里的mcurrentFrame使用的是指针，而在ORB-SLAM2中直接使用的是帧的实例
//     if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
//         mCurrentFrame.reset(new Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mClientId, mbf, mThDepth));
//         //mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
//     else
//     {
//         mCurrentFrame.reset(new Frame(mImGray,timestamp,mpORBextractor,mpORBVocabulary,mK,mDistCoef,mClientId, mbf, mThDepth));
//     }

//     Track();

//     return mCurrentFrame->mTcw.clone();
// }

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp,double &OPtimestamp,vector<vector<float>>&ObjectPostition)
{//这里需要修改
    

    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

        // 去畸变和极限对齐

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    mCurrentFrame.reset(new Frame(mImGray,imGrayRight,timestamp,mpORBextractor,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mClientId, mbf, mThDepth));
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    // cout<<"构造Frame++++++++++:"<<"总占时:"<<std::setprecision(6)<<ttrack<<endl;
    



    // Mat matlab_cameraMatrixL;
    // Mat matlab_distCoeffL;
    // Mat matlab_cameraMatrixR;
    // Mat matlab_distCoeffR;
    // Mat matlab_Rl, matlab_Rr, matlab_Pl, matlab_Pr, matlab_R, matlab_T;

    // const int imageWidth = 752;  //摄像头的分辨率  
    // const int imageHeight = 480;
    // Size imageSize = Size(imageWidth, imageHeight);
    // matlab_cameraMatrixL = (Mat_<double>(3, 3) << 367.7183527156617, 0, 382.701230211532,
	// 	0, 367.5502611526449, 244.8964736774738,
	// 	0, 0, 1);
	// matlab_distCoeffL = (Mat_<double>(5, 1) << -0.018194562804379723, -0.016791824828289788, 0.01972026694196412,-0.010638353029543805, 0.00000);
    
	// matlab_cameraMatrixR = (Mat_<double>(3, 3) << 367.4111540338462, 0, 368.3497399658523,
	// 	0, 367.2922877259461, 252.44528774757367,
	// 	0, 0, 1);
	// matlab_distCoeffR = (Mat_<double>(5, 1) << -0.019232312987564684, -0.017990786973418625, 0.026165178107054505,-0.01713608849313878, 0.00000);
	// matlab_R = (Mat_<double>(3, 3) << 0.9999805446276067, -0.005685795273660234, -0.002565560051687787,0.005681347626927274, 0.9999823514187759, -0.0017375672980619549, 0.00257539422512382, 0.0017229576545317188, 0.9999951993692286);
	// matlab_T = (Mat_<double>(3, 1) << -0.12006140637804055, -0.000712083342486687, -0.0003335105833742951);
    
    // Mat Rl, Rr, Pl, Pr, Q;
	
	// Rect validROIL;//
    // Rect validROIR;
	// //经过双目标定得到摄像头的各项参数后，采用OpenCV中的stereoRectify(立体校正)得到校正旋转矩阵R、投影矩阵P、重投影矩阵Q
	// //flags-可选的标志有两种零或者 CV_CALIB_ZERO_DISPARITY ,如果设置 CV_CALIB_ZERO_DISPARITY 的话，该函数会让两幅校正后的图像的主点有相同的像素坐标。否则该函数会水平或垂直的移动图像，以使得其有用的范围最大
	// //alpha-拉伸参数。如果设置为负或忽略，将不进行拉伸。如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
	// stereoRectify(matlab_cameraMatrixL, matlab_distCoeffL, matlab_cameraMatrixR, matlab_distCoeffR, imageSize, matlab_R, matlab_T, Rl, Rr, Pl, Pr, Q, CALIB_ZERO_DISPARITY,
	// 	-1, imageSize, &validROIL, &validROIR);
    // cout<<"*****************************************************************************"<<endl;
    // cout<<"Rl="<< Rl<<endl;
    // cout<<"Rr="<< Rr<<endl;
    // cout<<"Pl="<< Pl<<endl;
    // cout<<"Pr="<< Pr<<endl;
    // cout<<"*****************************************************************************"<<endl;
    long temp_time = long(timestamp*100 - 140363650000);
    if (mCurrentFrame->mId.first % 10 ==0)
    {
        cout<<"++++++++++++++++++++++++++++++++++++++++++++++++++++"<<endl;
        cout<<"第"<<mCurrentFrame->mId.first<<"帧时间戳 :"<<temp_time<<endl;
    }
    
    // 步骤3：跟踪
    Track();

    // // 这里为另一个客户端显示骨架点的代码
    // //zmf add
    // //cout<<mCurrentFrame->mTcw<<endl;
    // cv::Mat R(3,3,CV_32F);
    // cv::Mat pixel(3,1,CV_32F);//定义像素坐标系归一化坐标
    // //cv::Mat OP(3,1,CV_32F);
    // cv::Mat t(3,1,CV_32F);
    // cv::Mat r_vec(3,1,CV_32F);

    // //OP.at<float>(0) = OP_x;
    // //OP.at<float>(1) = OP_y;
    // //OP.at<float>(2) = OP_z;
    // if(!mCurrentFrame->mTcw.empty()&&mClientId==1&&!ObjectPostition.empty())//如果当前帧的Tcw不为空并且客户端id为1
    // {
    //     //////////////////////////////////////////////
    //     //这一段代码是为了获取相机的位姿R和t

    //     Eigen::Matrix3d R;
    //     R<<mCurrentFrame->mTcw.at<float>(0,0),mCurrentFrame->mTcw.at<float>(0,1),mCurrentFrame->mTcw.at<float>(0,2),
    //         mCurrentFrame->mTcw.at<float>(1,0),mCurrentFrame->mTcw.at<float>(1,1),mCurrentFrame->mTcw.at<float>(1,2),
    //         mCurrentFrame->mTcw.at<float>(2,0),mCurrentFrame->mTcw.at<float>(2,1),mCurrentFrame->mTcw.at<float>(2,2);
        
    //     // cv::Mat mat_R(3,3,CV_32F);//Matrix3d 转cv mat
    //     // for(int i=0;i<3;i++)
    //     //     for(int j=0; j<3; j++)
    //     //         mat_R.at<float>(i,j)=R(i,j);

    //     t.at<float>(0) = mCurrentFrame->mTcw.at<float>(3,0);
    //     t.at<float>(1) = mCurrentFrame->mTcw.at<float>(3,1);
    //     t.at<float>(2) = mCurrentFrame->mTcw.at<float>(3,2);

    //     double theta = acos((R.trace() - 1) * 0.5);	//待求的旋转向量的角度

    //     Eigen::Matrix3d Right = (R - R.transpose()) * 0.5 / sin(theta);
    //     Eigen::Vector3d R_vec;	//待求的旋转向量的单位向量
    //     R_vec[0] = (Right(2,1) - Right(1,2))*0.5;
    //     R_vec[1] = (Right(0,2) - Right(2,0))*0.5;
    //     R_vec[2] = (Right(1,0) - Right(0,1))*0.5;
    //     r_vec.at<float>(0) = R_vec[0]*theta;
    //     r_vec.at<float>(1) = R_vec[1]*theta;
    //     r_vec.at<float>(2) = R_vec[2]*theta;
        
    //     //cout<<"r_vec="<<r_vec.t()<<", "<<theta<<endl;

    //     Eigen::AngleAxisd rotation_vector(R);

    //     auto a = rotation_vector.axis();
    //     auto b = rotation_vector.angle();
    //     //cout<<"rotation_vector"<<b*a.transpose()<<", "<<b<<endl;


    //     // cvRodrigues2(mat_R,r_vec,0);
    //     //cout<<"关键物体像素坐标计算"<<endl;
    //     //cout<<"R: "<<endl<<R<<endl;
    //     ///////////////////////////////////////////////
    //     //ObjectPostition
    //     vector<cv::Point3f> points;//存放关键点三维坐标
    //     vector<cv::Point2f> pixel_uv;//存放三维坐标投影之后的二维像素坐标
    //     for(int i=0;i<ObjectPostition.size();i++)

    //     {   
    //         //传过来的数据没问题
    //         //cout<<"++++++++++++++++++++ObjectPostition x y z="<<ObjectPostition[i][0]<<" "<<ObjectPostition[i][1]<<" "<<ObjectPostition[i][2]<<endl;
    //         points.push_back(cv::Point3f(ObjectPostition[i][0],ObjectPostition[i][1],ObjectPostition[i][2]));//添加三维坐标
    //     }

    //     cv::fisheye::projectPoints(points, pixel_uv,r_vec,t,mK,mDistCoef);//调用投影函数
    //     //cv::projectPoints(points,r_vec,t,mK,mDistCoef,pixel_uv);//调用投影函数
    //     //points 为三维坐标 pixel_uv为二维坐标 R为旋转矩阵 mK为相机内参矩阵 mDistCoef为畸变参数 
    //     //pixel=mK*(R*OP+t);

    //     //cv::circle(src, point, 4, cv::Scalar(0, 0, 255));//在图像中画出特征点，2是圆的半径 
    //     cv::cvtColor(mImGray, mImGray, COLOR_GRAY2RGB);//灰度图像转彩色图像
    //     //string OPtext="("+to_string(OP_x)+","+to_string(OP_y)+","+to_string(OP_z)+")";
    //     for(int i=0;i<pixel_uv.size();i++)
    //     {
    //         //float u,v;
    //         //int squ_size=28;//正方形框的边长
    //         //u=pixel_uv[i].x;
    //         //v=pixel_uv[i].y;
    //         // todo: 加畸变
    //         //cout<<"pixel_uv="<<pixel_uv<<endl;
    //         //cout<<"像素坐标 u="<<u<<endl;
    //         //cout<<"像素坐标 v="<<v<<endl;
    //         cv::circle(mImGray, pixel_uv[i], 4, cv::Scalar(0, 0, 255));

    //     }
    //     cv::imshow("image",mImGray);
    //    cv::waitKey(25);     
//}
    
    // cout<<"输出位姿+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<endl;
    /////////////////end add ////////////////////////////////////
    return mCurrentFrame->mTcw.clone(); 
}

cv::Mat Tracking::GrabImageMultiple(const vector<cv::Mat>&vImages, const double &timestamp, double &OPtimestamp,vector<vector<float>>&ObjectPostition)
{
    
    vector<cv::Mat> vImRect(vImages.begin(), vImages.end());
    mImGray = vImages[0];
    cout<<"GrabImageMultiple camera num: "<<vImages.size()<<endl;
    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            for(int i = 0; i < cameraNum; i++)
            {
                cvtColor(vImRect[i],vImRect[i],CV_RGB2GRAY);
            }
        }
        else
        {
             for(int i = 0; i < cameraNum; i++)
            {
                cvtColor(vImRect[i],vImRect[i],CV_BGR2GRAY);
            }
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            for(int i = 0; i < cameraNum; i++)
            {
                cvtColor(vImRect[i],vImRect[i],CV_RGB2GRAY);
            }
        }
        else
        {
            for(int i = 0; i < cameraNum; i++)
            {
                cvtColor(vImRect[i],vImRect[i],CV_BGR2GRAY);
            }
        }
    }

        // 去畸变和极限对齐

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    mCurrentFrame.reset(new Frame(vImRect,timestamp,mvpORBextractorMultiple,mpORBVocabulary,mvK,mvDistCoef,mClientId, mThDepth));
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

 //   long temp_time = long(timestamp*100 - 140363650000);
    // if (mCurrentFrame->mId.first% 10 ==0) // todo do not forget uncomment
    // {
        // cout<<"++++++++++++++++++++++++++++++++++++++++++++++++++++"<<endl;
        // cout<<"第"<<mCurrentFrame->mId.first<<"帧时间戳 :"<<temp_time<<endl;
    //} //end todo
    
    // 步骤3：跟踪
    Track();
//todo 
//     // 这里为另一个客户端显示骨架点的代码
//     //zmf add
    
//     cv::Mat R(3,3,CV_32F);
//     cv::Mat pixel(3,1,CV_32F);//定义像素坐标系归一化坐标
   
//     cv::Mat t(3,1,CV_32F);
//     cv::Mat r_vec(3,1,CV_32F);

    
//     if(!mCurrentFrame->mTcw.empty()&&mClientId==1&&!ObjectPostition.empty())//如果当前帧的Tcw不为空并且客户端id为1
//     {


//         Eigen::Matrix3d R;
//         R<<mCurrentFrame->mTcw.at<float>(0,0),mCurrentFrame->mTcw.at<float>(0,1),mCurrentFrame->mTcw.at<float>(0,2),
//             mCurrentFrame->mTcw.at<float>(1,0),mCurrentFrame->mTcw.at<float>(1,1),mCurrentFrame->mTcw.at<float>(1,2),
//             mCurrentFrame->mTcw.at<float>(2,0),mCurrentFrame->mTcw.at<float>(2,1),mCurrentFrame->mTcw.at<float>(2,2);
    

//         t.at<float>(0) = mCurrentFrame->mTcw.at<float>(3,0);
//         t.at<float>(1) = mCurrentFrame->mTcw.at<float>(3,1);
//         t.at<float>(2) = mCurrentFrame->mTcw.at<float>(3,2);

//         double theta = acos((R.trace() - 1) * 0.5);	//待求的旋转向量的角度

//         Eigen::Matrix3d Right = (R - R.transpose()) * 0.5 / sin(theta);
//         Eigen::Vector3d R_vec;	//待求的旋转向量的单位向量
//         R_vec[0] = (Right(2,1) - Right(1,2))*0.5;
//         R_vec[1] = (Right(0,2) - Right(2,0))*0.5;
//         R_vec[2] = (Right(1,0) - Right(0,1))*0.5;
//         r_vec.at<float>(0) = R_vec[0]*theta;
//         r_vec.at<float>(1) = R_vec[1]*theta;
//         r_vec.at<float>(2) = R_vec[2]*theta;
    

//         Eigen::AngleAxisd rotation_vector(R);

//         auto a = rotation_vector.axis();
//         auto b = rotation_vector.angle();

//         vector<cv::Point3f> points;//存放关键点三维坐标
//         vector<cv::Point2f> pixel_uv;//存放三维坐标投影之后的二维像素坐标
//         for(int i=0;i<ObjectPostition.size();i++)

//         {   
      
//             points.push_back(cv::Point3f(ObjectPostition[i][0],ObjectPostition[i][1],ObjectPostition[i][2]));//添加三维坐标
//         }

//         cv::fisheye::projectPoints(points, pixel_uv,r_vec,t,mK,mDistCoef);//调用投影函数
 
//         cv::cvtColor(mImGray, mImGray, COLOR_GRAY2RGB);//灰度图像转彩色图像
   
//         for(int i=0;i<pixel_uv.size();i++)
//         {
//             cv::circle(mImGray, pixel_uv[i], 4, cv::Scalar(0, 0, 255));

//         }
//         cv::imshow("image",mImGray);
//         cv::waitKey(25);     
// }
//endtodo uncomment
    return mCurrentFrame->mTcw.clone(); 

}


void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }


    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    while(!mpBMap->LockBMapUpdate()){
        usleep(params::timings::miLockSleep);
    }
    //Comm Mutex cannot be acquired here. In case of wrong initialization, there is mutual dependency in the call of reset()
    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==CentralControl::MULTIPLE)
            MultiCameraInitialization();
        else if(mSensor == CentralControl::STEREO)
            StereoInitialization(); //需要加入双目初始化
        else
            MonocularInitialization();

        if(params::vis::mbActive)//==0  不进
            mpViewer->UpdateAndDrawFrame(); //get the errors  11.20
        else // FIXME: 这里少了一个else
        {
            //cout << "\033[1;35m!!! +++ Tracking: Init +++ !!!\033[0m" << endl;
        }

        if(mState!=OK)
        {
            mpBMap->UnLockBMapUpdate();
            return;
        }
    }
    else
    {
        // cout<<"Frame: "<<++count<<" Initialization successfully!"<<endl;
        // Get Communicator Mutex -> Comm cannot publish. Assure no publishing whilst changing data
        if(params::sys::mbStrictLock) while(!mpCC->LockTracking()){
            usleep(params::timings::miLockSleep);
        }

        // System is initialized. Track Frame.
        bool bOK;


        //少了一个纯定位的模式，只有正常VO的过程
        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(mState==OK) //normal initial 
        {
            // Local Mapping might have changed some MapPoints tracked in last frame
            CheckReplacedInLastFrame();

            if(mVelocity.empty() || mCurrentFrame->mId.first<mLastRelocFrameId.first+2)
            {
                    
                    //std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                    bOK = TrackReferenceBundledKeyFrames();
                    //std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                    // ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
                    //cout<<"TrackReferenceKeyFrame++++++++++: "<<bOK<<endl;
            }
            else
            {
                   // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                    bOK = TrackWithMotionModel();
                   // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                    //double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
                    //cout<<"Motion model++++++++++: "<<bOK<<endl;

                if(!bOK)
                {

                    //std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                    bOK = TrackReferenceBundledKeyFrames();

                    //std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                    //double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
                    //cout<<"motion不行 TrackReferenceKeyFrame++++++++++: "<<bOK<<endl;
                
                }
            }
        }
        else
        {

            cout << "\033[1;35m!!! +++ Tracking: Lost +++ !!!\033[0m" <<mCurrentFrame->mId.first<< endl;
            bOK = false; // 没有重定位过程
        }

        mCurrentFrame->mpReferenceBKFs = mpReferenceBKFs;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(bOK)
        {
           // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            bOK = TrackLocalBMap(); 
            //std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            //double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            //cout<<"TrackLocalMap++++++++++:"<<bOK<<endl;
        } 

        if(bOK)
            mState = OK;
        else
            mState=LOST;
            

        // Update drawer
        if(params::vis::mbActive) mpViewer->UpdateAndDrawFrame();

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            //cout<<"跟踪情况良好++++++++++"<<endl;
            // Update motion model
            if(!mLastFrame->mTcw.empty())
            {
                //cout<<"如果上一帧位姿不为空，更新匀速运动模型速度++++++++++"<<endl;
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame->GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame->GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame->mTcw*LastTwc; //Tcl, the roration betweent last frame and  current frame
            }
            else
            {
                //cout<<"如果上一帧位姿为空，将匀速运动模型速度设置为空++++++++++"<<endl;
                mVelocity = cv::Mat();
            }
                

            // Clean VO matches
            //清除UpdateLastFrame中为当前帧临时添加的MapPoints
            for(int i=0; i<mCurrentFrame->N_L_R; i++)
            {
                mpptr pMP = mCurrentFrame->mvpMapPointsBKFs[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame->mvbOutlierBKFs[i] = false;
                        mCurrentFrame->mvpMapPointsBKFs[i]=nullptr;
                    }
            }

            //清除用恒速度双目跟踪中，生成的临时地图点，这些点时在UpdateLastFrame函数中生成的；这些点只是为了提高跟踪精度，并不要用于建图
            for(list<mpptr>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                mpptr pMp = *lit; // 需要再次确定FIXME:
                pMp.reset();  //对之前在地图点中的指针也进行释放,即比较彻底的删除了
                // delete pMp; 
            }

            mlpTemporalPoints.clear();
            // Check if we need to insert a new keyframe
            if(NeedNewBundledKeyFrames())
            {
                //cout<<"需要创建关键帧++++++++++"<<endl;
                CreateNewBundledKeyFrames();
            }
                

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame->N_L_R;i++)
            {
                if(mCurrentFrame->mvpMapPointsBKFs[i] && mCurrentFrame->mvbOutlierBKFs[i])
                    mCurrentFrame->mvpMapPointsBKFs[i]=nullptr;
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpBMap->BundledKeyFramesInMap()<=params::tracking::miInitKFs)//5
            {
                cout<<"    RESET时具有"<<mpBMap->BundledKeyFramesInMap()<<" BundledKeyFrames "<<endl;
                //cout << "Track lost soon after initialisation, reseting..." << endl;
                if(params::sys::mbStrictLock) mpCC->UnLockTracking();
                mpBMap->UnLockBMapUpdate();
                mpCC->mpCH->Reset(); //clientHandler调用reset()对系统进行了重启

                return;
            }
        }

        if(!mCurrentFrame->mpReferenceBKFs)
            mCurrentFrame->mpReferenceBKFs = mpReferenceBKFs;

        mLastFrame.reset(new Frame(*mCurrentFrame));

        if(params::sys::mbStrictLock) mpCC->UnLockTracking();
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame->mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame->mTcw*mCurrentFrame->mpReferenceBKFs->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferencesBKFs.push_back(mpReferenceBKFs);
        mlFrameTimes.push_back(mCurrentFrame->mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferencesBKFs.push_back(mlpReferencesBKFs.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

    mpBMap->UnLockBMapUpdate();
}

//multi-camera
void Tracking::MultiCameraInitialization()
{
    cout<<"Enter MultiCameraInitialization 1"<<endl;
    int flag = 0;
    for(int i = 0; i < cameraNum-1; i++)
    {
        if(mCurrentFrame->mvpkeyPointsNum[i] > 500)
            ++flag;
    }
    if(flag == cameraNum-1)
    {
        mCurrentFrame->SetPose(cv::Mat::eye(4,4,CV_32F));

        // vector<kfptr> tvKeyFrames;
        // tvKeyFrames.reserve(cameraNum);

        // //create keyframe which is subbundledkeyframe
        // int frameId = -1;
        // for(int i = 0; i < cameraNum; i++)
        // {
        //     if(i == 0)
        //     {
        //         kfptr pKFini{new KeyFrame(*mCurrentFrame,frameId, i, eSystemState::CLIENT,-1)};
        //         frameId = pKFini->mId.first;
        //         tvKeyFrames.push_back(pKFini);
        //     }
        //     else
        //     {
        //         kfptr pKFini{new KeyFrame(*mCurrentFrame,frameId, i, eSystemState::CLIENT,-1)};
        //         tvKeyFrames.push_back(pKFini);
        //     }
        // }
        
        bkfptr pBKFsini{new BundledKeyFrames(*mCurrentFrame, mpBMap, mpBundledKeyFramesDB, mpComm, eSystemState::CLIENT,-1 )};
        mpBMap->AddBundledKeyFrames(pBKFsini);
        int count = 0;
        for(int i=0; i<mCurrentFrame->N_L_R;i++)
        {
            float z = mCurrentFrame->mvBDepth[i];

            if(z>0)
            {
                ++count;
                cv::Mat x3D = mCurrentFrame->ComputeMapPointInWorldFrame(mCurrentFrame->mvx3Ds[i].mPos);

                //cout<<x3D<<endl;
                mpptr pNewMP{new MapPoint(x3D, pBKFsini, mpBMap, mClientId, mpComm, eSystemState::CLIENT,-1)};
                if(!pNewMP)
                    cerr << "create mappoint failed"<<endl;
                pNewMP->AddBKFsObservation(pBKFsini,i, cameraNum);
      
                pBKFsini->AddMapPoint(pNewMP, i);

                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepthPlus();

                mpBMap->AddMapPoint(pNewMP);

                mCurrentFrame->mvpMapPointsBKFs[i]=pNewMP;

                
            }
        }



        cout<<count<<endl;
        cout << "New map created with " << mpBMap->MapPointsInMap() << " points" << endl;
        mpLocalMapper->InsertBundledKeyFrames(pBKFsini); //todo

        mLastFrame.reset(new Frame(*mCurrentFrame)); 

        mLastBundledKeyFramesId=mCurrentFrame->mId;

        mpLastBundledKeyFrames = pBKFsini;

        mvpLocalBundledKeyFrames.push_back(pBKFsini);

        mvpLocalMapPoints=mpBMap->GetAllMapPoints();

        mpReferenceBKFs = pBKFsini;

        mCurrentFrame->mpReferenceBKFs = pBKFsini;

        mpBMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpBMap->mvpBundledKeyFramesOrigins.push_back(pBKFsini);

        mState=OK;
        //for debug
        // int mpcount =0;
        // for(auto mp:mpReferenceBKFs->GetMapPointMatches())
        // {
        //     if(mp)
        //         ++mpcount;
        // }
        //end debug
        cout<<"Multi camera Initialization successfully!"<<endl;
    }
}



//加入双目的初始化函数
void Tracking::StereoInitialization()
{
    // if(mCurrentFrame->N > 500)
    // {
    //     //set Frame pose to the origin. indentity matrix
    //     mCurrentFrame->SetPose(cv::Mat::eye(4, 4, CV_32F));

    //     //create the keyframes
    //     kfptr pKFini{new KeyFrame(*mCurrentFrame,mpMap,mpKeyFrameDB,mpComm,eSystemState::CLIENT,-1)};
    //     //insert the keyframe
    //     mpMap->AddKeyFrame(pKFini);
    //     //create Mappoints and associate to keyframe
    //     //测试一下能够正确算出地图点深度的特征
    //     // char *output = "/home/slam/Desktop/ccmslam.txt";
    //     // ofstream out(output);
    //     // for(int i=0; i < mCurrentFrame->N; i++) {
    //     //     if (mCurrentFrame->mvDepth[i] > 0) {

    //     //         //std::cout << mCurrentFrame->mId.first << " " << mCurrentFrame->mvKeysUn[i].pt.x
    //     //          //         << mCurrentFrame->mvKeysUn[i].pt.y << mCurrentFrame->mvDepth[i] << std::endl;
    //     //         float u = mCurrentFrame->mvKeysUn[i].pt.x;
    //     //         float v = mCurrentFrame->mvKeysUn[i].pt.y;
    //     //         float depth = mCurrentFrame->mvDepth[i];
    //     //         out << u << ", " << v << ", " << depth << std::endl;
    //     //     }
    //     //    // std::cout << "the dh is : " << mCurrentFrame->mvDepth[i] << std::endl;
    //     //     //std::cout << "the number of features is :" << mCurrentFrame->N << std::endl;
    //     // }
    //     // out.close();
    //     for(int i=0; i < mCurrentFrame->N; i++)
    //     {
    //         float z = mCurrentFrame->mvDepth[i];

    //         if(z > 0)
    //         {
    //             cv:Mat x3D = mCurrentFrame->UnprojectStereo(i);
    //             mpptr pNewMP{new MapPoint(x3D, pKFini, mpMap,mClientId,mpComm,eSystemState::CLIENT,-1)}; //加入CCM中地图点类构造函数具有的成员函数
    //             pNewMP->AddObservation(pKFini, i);  //有区别于单双目相机
    //             pNewMP->ComputeDistinctiveDescriptors();
    //             pNewMP->UpdateNormalAndDepth();
    //             mpMap->AddMapPoint(pNewMP);
    //             //查看该keyframe的哪个特征点能看到该3D点
    //             pKFini->AddMapPoint(pNewMP,i);
    //             mCurrentFrame->mvpMapPoints[i] = pNewMP;

    //         }
    //     }


    //     mpLocalMapper->InsertKeyFrame(pKFini);

    //     mLastFrame.reset(new Frame(*mCurrentFrame)); 
    //     mLastKeyFrameId=mCurrentFrame->mId;
    //     mpLastKeyFrame = pKFini;
    //     mvpLocalKeyFrames.push_back(pKFini);
    //     mvpLocalMapPoints=mpMap->GetAllMapPoints();
    //     mpReferenceKF = pKFini;
    //     mCurrentFrame->mpReferenceKF = pKFini;

    //     mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
    //     mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    //     mState = OK;
    //     // mpCC->UnlockTracking();

    // }
}

void Tracking::MonocularInitialization()
{
    // cout<<"单目初始化++++++++++"<<endl;
    // if(!mpInitializer)
    // {
    //     // Set Reference Frame
    //     if(mCurrentFrame->mvKeys.size()>100)
    //     {
    //         mInitialFrame.reset(new Frame(*mCurrentFrame));
    //         mLastFrame.reset(new Frame(*mCurrentFrame));
    //         mvbPrevMatched.resize(mCurrentFrame->mvKeysUn.size());
    //         for(size_t i=0; i<mCurrentFrame->mvKeysUn.size(); i++)
    //             mvbPrevMatched[i]=mCurrentFrame->mvKeysUn[i].pt;

    //         if(mpInitializer) mpInitializer = nullptr;

    //         mpInitializer.reset(new Initializer(*mCurrentFrame,1.0,200));

    //         fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

    //         return;
    //     }
    // }
    // else
    // {
    //     // Try to initialize
    //     if((int)mCurrentFrame->mvKeys.size()<=100)
    //     {
    //         mpInitializer = nullptr;
    //         fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
    //         return;
    //     }

    //     // Find correspondences
    //     ORBmatcher matcher(0.9,true);
    //     int nmatches = matcher.SearchForInitialization(*mInitialFrame,*mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

    //     // Check if there are enough correspondences
    //     if(nmatches<100)
    //     {
    //         //delete mpInitializer;
    //         mpInitializer = nullptr;
    //         return;
    //     }

    //     cv::Mat Rcw; // Current Camera Rotation
    //     cv::Mat tcw; // Current Camera Translation
    //     vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

    //     if(mpInitializer->Initialize(*mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
    //     {
    //         for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
    //         {
    //             if(mvIniMatches[i]>=0 && !vbTriangulated[i])
    //             {
    //                 mvIniMatches[i]=-1;
    //                 nmatches--;
    //             }
    //         }

    //         // Set Frame Poses
    //         mInitialFrame->SetPose(cv::Mat::eye(4,4,CV_32F));
    //         cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
    //         Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
    //         tcw.copyTo(Tcw.rowRange(0,3).col(3));
    //         mCurrentFrame->SetPose(Tcw);

    //         CreateInitialMapMonocular();
    //     }
    // }
}

void Tracking::CreateInitialMapMonocular()
{
    // // Get Communicator Mutex -> Comm cannot publish. Assure no publishing whilst changing data
    // while(!mpCC->LockTracking()){
    //     usleep(params::timings::miLockSleep);
    // }

    // // Create KeyFrames
    // kfptr pKFini{new KeyFrame(*mInitialFrame,mpMap,mpKeyFrameDB,mpComm,eSystemState::CLIENT,-1)};
    // kfptr pKFcur{new KeyFrame(*mCurrentFrame,mpMap,mpKeyFrameDB,mpComm,eSystemState::CLIENT,-1)};

    // pKFini->ComputeBoW();
    // pKFcur->ComputeBoW();

    // // Insert KFs in the map
    // mpMap->AddKeyFrame(pKFini);
    // mpMap->AddKeyFrame(pKFcur);

    // // Create MapPoints and asscoiate to keyframes
    // for(size_t i=0; i<mvIniMatches.size();i++)
    // {
    //     if(mvIniMatches[i]<0)
    //         continue;

    //     //Create MapPoint.
    //     cv::Mat worldPos(mvIniP3D[i]);

    //     mpptr pMP{new MapPoint(worldPos,pKFcur,mpMap,mClientId,mpComm,eSystemState::CLIENT,-1)};

    //     pKFini->AddMapPoint(pMP,i);
    //     pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

    //     pMP->AddObservation(pKFini,i);
    //     pMP->AddObservation(pKFcur,mvIniMatches[i]);

    //     pMP->ComputeDistinctiveDescriptors();
    //     pMP->UpdateNormalAndDepth();

    //     //Fill Current Frame structure
    //     mCurrentFrame->mvpMapPoints[mvIniMatches[i]] = pMP;
    //     mCurrentFrame->mvbOutlier[mvIniMatches[i]] = false;

    //     //Add to Map
    //     mpMap->AddMapPoint(pMP);
    // }

    // // Update Connections
    // pKFini->UpdateConnections();
    // pKFcur->UpdateConnections();

    // // Bundle Adjustment

    // Optimizer::GlobalBundleAdjustemntClient(mpMap,mClientId,20);

    // // Set median depth to 1
    // float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    // float invMedianDepth = 1.0f/medianDepth;

    // if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    // {
    //     cout << "Wrong initialization, reseting..." << endl;
    //     mpCC->UnLockTracking();
    //     Reset();
    //     return;
    // }

    // // Scale initial baseline
    // cv::Mat Tc2w = pKFcur->GetPose();
    // Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    // pKFcur->SetPose(Tc2w,false);

    // // Scale points
    // vector<mpptr> vpAllMapPoints = pKFini->GetMapPointMatches();

    // for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    // {
    //     if(vpAllMapPoints[iMP])
    //     {
    //         mpptr pMP = vpAllMapPoints[iMP];
    //         pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth,false);
    //     }
    // }

    // mpLocalMapper->InsertKeyFrame(pKFini);
    // mpLocalMapper->InsertKeyFrame(pKFcur);

    // mCurrentFrame->SetPose(pKFcur->GetPose());
    // mLastKeyFrameId=mCurrentFrame->mId;
    // mpLastKeyFrame = pKFcur;

    // mvpLocalKeyFrames.push_back(pKFcur);
    // mvpLocalKeyFrames.push_back(pKFini);
    // mvpLocalMapPoints=mpMap->GetAllMapPoints();
    // mpReferenceKF = pKFcur;
    // mCurrentFrame->mpReferenceKF = pKFcur;

    // mLastFrame.reset(new Frame(*mCurrentFrame));

    // mpMap->SetReferenceMapPoints(mvpLocalMapPoints);


    // mpMap->mvpKeyFrameOrigins.push_back(pKFini);
        

    // mState=OK;

    // mpCC->UnLockTracking();
 }

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame->N_L_R; i++)
    {
        mpptr pMP = mLastFrame->mvpMapPointsBKFs[i];

        if(pMP)
        {
            mpptr pRep = pMP->GetReplaced();//更新地图点的信息
            if(pRep)
            {
                 mLastFrame->mvpMapPointsBKFs[i] = pRep;
            }
        }
    }
}



bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame->ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<mpptr> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,*mCurrentFrame,vpMapPointMatches);
    
    if(nmatches<params::tracking::miTrackWithRefKfInlierThresSearch)//15
        return false;

    mCurrentFrame->mvpMapPoints = vpMapPointMatches;
    cout<<"ReferenceKF 匹配 map points个数"<<vpMapPointMatches.size()<<endl;
    mCurrentFrame->SetPose(mLastFrame->mTcw);

    Optimizer::PoseOptimizationClient(*mCurrentFrame);
   

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame->N; i++)
    {
        if(mCurrentFrame->mvpMapPoints[i])
        {
            if(mCurrentFrame->mvbOutlier[i])
            {
                mpptr pMP = mCurrentFrame->mvpMapPoints[i];

                mCurrentFrame->mvpMapPoints[i]=nullptr;
                mCurrentFrame->mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mLastFrameSeen = mCurrentFrame->mId;
                nmatches--;
            }
            else if(mCurrentFrame->mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=params::tracking::miTrackWithRefKfInlierThresOpt; //10
}

bool Tracking::TrackReferenceBundledKeyFrames()
{
    // Compute Bag of Words vector
    mCurrentFrame->ComputeBoW();

    mpReferenceBKFs->ComputeBoW();
    //cout<<"=================chose TrackReferenceBundledKeyFrames model========"<<endl;
    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<mpptr> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceBKFs,*mCurrentFrame,vpMapPointMatches);

    if(nmatches<params::tracking::miTrackWithRefKfInlierThresSearch)//15
        return false;

    mCurrentFrame->mvpMapPointsBKFs = vpMapPointMatches;
    cout<<"BundledreferenceKF 匹配 map points个数"<<nmatches<<endl;
    mCurrentFrame->SetPose(mLastFrame->mTcw);

    Optimizer::PoseOptimizationClientPlus(*mCurrentFrame);
   

    // Discard outliers
    int nmatchesMap = 0;
    int mpcount = 0;
    for(int i =0; i<mCurrentFrame->N_L_R; i++)
    {
        if(mCurrentFrame->mvpMapPointsBKFs[i])
        {
            ++mpcount;
            if(mCurrentFrame->mvbOutlierBKFs[i])
            {
                mpptr pMP = mCurrentFrame->mvpMapPointsBKFs[i];

                mCurrentFrame->mvpMapPointsBKFs[i]=nullptr;
                mCurrentFrame->mvbOutlierBKFs[i]=false;
                pMP->mbTrackInView = false;
                pMP->mLastFrameSeen = mCurrentFrame->mId;
                nmatches--;
            }
            else if(mCurrentFrame->mvpMapPointsBKFs[i]->Observations()>0)
                nmatchesMap++;
        }
    }
    cout<<"mappoint size: "<<mpcount<<endl;
    return nmatchesMap>=params::tracking::miTrackWithRefKfInlierThresOpt; //10
}

void Tracking::UpdateLastFrame() //add some temporal map points 
{
    // Update pose according to reference keyframe
    bkfptr pRef = mLastFrame->mpReferenceBKFs;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame->SetPose(Tlr*pRef->GetPose());
    //单目,以及上一帧没关键帧的话,不需要该操作
    if(mLastBundledKeyFramesId == mLastFrame->mId || mSensor == CentralControl::MONOCULAR)
        return;
   
    // 步骤2：对于双目或rgbd摄像头，为上一帧临时生成新的MapPoints
    // 注意这些MapPoints不加入到Map中，在tracking的最后会删除
    // 跟踪过程中需要将将上一帧的MapPoints投影到当前帧可以缩小匹配范围，加快当前帧与上一帧进行特征点匹配
    // 步骤2.1：得到上一帧有深度值的特征点
    vector<pair<float, int>> vDepthIdx;
    vDepthIdx.reserve(mLastFrame->N_L_R);

    for(int i=0; i<mLastFrame->N_L_R; i++) 
    {
        float z = mLastFrame->mvBDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }
    if(vDepthIdx.empty())
        return;
    // 步骤2.2：按照深度从小到大排序
    sort(vDepthIdx.begin(),vDepthIdx.end());

    //2.3:近点可用于包装成地图点
    int nPoints = 0;
    for(size_t j =0; j < vDepthIdx.size(); j++)
    {
        int i = vDepthIdx[j].second;
        bool bCreateNew = false;

        mpptr pMP = mLastFrame->mvpMapPointsBKFs[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
             bCreateNew = true;
        }

        if(bCreateNew)
        { 
            cv::Mat x3D = mLastFrame->ComputeMapPointInWorldFrame(mLastFrame->mvx3Ds[i].mPos);
            mpptr pNewMP{new MapPoint(x3D, mpBMap, mLastFrame, mLastFrame->mvx3Ds[i].cameraId, i, mClientId)};
            mLastFrame->mvpMapPointsBKFs[i]=pNewMP;
            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }
        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;        
        
    }
    cout<<"Enter UpdateLastFrame end"<<endl;
}

bool Tracking::TrackWithMotionModel()
{
    
    //cout<<"=================chose trackwithmotion model========"<<endl;
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();
    
    mCurrentFrame->SetPose(mVelocity*mLastFrame->mTcw);
    
    fill(mCurrentFrame->mvpMapPointsBKFs.begin(),mCurrentFrame->mvpMapPointsBKFs.end(),nullptr);

    // Project points seen in previous frame
    int th;
    th=7; //在ORB中双目的是7，单目的是15，但是这里直接设置成了7
        // <时间>
    int nmatches = matcher.SearchByProjectionPlus(*mCurrentFrame,*mLastFrame,th);
        // </时间>
  
    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame->mvpMapPointsBKFs.begin(),mCurrentFrame->mvpMapPointsBKFs.end(),nullptr);
        nmatches = matcher.SearchByProjectionPlus(*mCurrentFrame,*mLastFrame,2*th);
    }
    cout<<"Enter TrackWithMotionMode matches size: "<<nmatches<<endl;
    if(nmatches<params::tracking::miTrackWithMotionModelInlierThresSearch) //20
        return false;

    // Optimize frame pose with all matches
        // <时间>
    Optimizer::PoseOptimizationClientPlus(*mCurrentFrame);
        // </时间>

    int nmatchesMap = 0;
    int mpcount = 0;
    for(int i =0; i<mCurrentFrame->N_L_R; i++)
    {
        if(mCurrentFrame->mvpMapPointsBKFs[i])
        {
            ++mpcount;
            if(mCurrentFrame->mvbOutlierBKFs[i])
            {
                mpptr pMP = mCurrentFrame->mvpMapPointsBKFs[i];

                mCurrentFrame->mvpMapPointsBKFs[i]=nullptr;
                mCurrentFrame->mvbOutlierBKFs[i]=false;
                pMP->mbTrackInView = false;
                pMP->mLastFrameSeen = mCurrentFrame->mId;
                nmatches--;
            }
            else if(mCurrentFrame->mvpMapPointsBKFs[i]->Observations()>0)
                nmatchesMap++;
        }
    }
    cout<<"mappoint size: "<<mpcount<<endl;
    return nmatchesMap>=params::tracking::miTrackWithMotionModelInlierThresOpt; //10
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap(); //update the local keyframes and points 

    SearchLocalPoints(); //track the map points by the projecting in the current frames 

    // Optimize Pose
    //cout<<"在TrackLocalMap函数中调用 PoseOptimizationClient优化函数++++++++++"<<endl;
    Optimizer::PoseOptimizationClient(*mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame->N; i++)
    {
        if(mCurrentFrame->mvpMapPoints[i])
        {
            if(!mCurrentFrame->mvbOutlier[i])
            {
                mCurrentFrame->mvpMapPoints[i]->IncreaseFound();
                if(mCurrentFrame->mvpMapPoints[i]->Observations()>0) //FIXME:
                        mnMatchesInliers++;
            }
            else if (mSensor == CentralControl::STEREO) //对于双目而言,直接对这些outlier进行删除
                mCurrentFrame->mvpMapPoints[i] = nullptr;

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame->mId.first<mLastRelocFrameId.first+params::tracking::miMaxFrames && mnMatchesInliers<50)
    {
        return false;
    }

    if(mnMatchesInliers<params::tracking::miTrackLocalMapInlierThres) //30
        return false;
    else
        return true;
}

bool Tracking::TrackLocalBMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalBMap(); //update the local keyframes and points 

    SearchLocalPointsPlus(); //track the map points by the projecting in the current frames 

    // Optimize Pose
    //cout<<"在TrackLocalMap函数中调用 PoseOptimizationClient优化函数++++++++++"<<endl;
    Optimizer::PoseOptimizationClientPlus(*mCurrentFrame);
    mnMatchesInliers = 0;
    int mpcount = 0;
    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame->N_L_R; i++)
    {
        if(mCurrentFrame->mvpMapPointsBKFs[i])
        {
            ++mpcount;
            if(!mCurrentFrame->mvbOutlierBKFs[i])
            {
                mCurrentFrame->mvpMapPointsBKFs[i]->IncreaseFound();
                if(mCurrentFrame->mvpMapPointsBKFs[i]->Observations()>0) //FIXME:
                        mnMatchesInliers++;
            }
            else if (mSensor == CentralControl::STEREO) //对于双目而言,直接对这些outlier进行删除
                mCurrentFrame->mvpMapPointsBKFs[i] = nullptr;

        }
    }
    cout<<"Track bmap mappoint size: "<<mpcount<<endl;
    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame->mId.first<mLastRelocFrameId.first+params::tracking::miMaxFrames && mnMatchesInliers<50)
    {
        cout<<"(track local Bmap) mnMatchesInliers1: "<<mnMatchesInliers<<endl;
        return false;
    }

    if(mnMatchesInliers<30) //todo： 30 params::tracking::miTrackLocalMapInlierThres
    {
        cout<<"(track local Bmap) mnMatchesInliers: "<<mnMatchesInliers<<endl;
        return false;
    }
    else
        return true;
}

bool Tracking::NeedNewKeyFrame()
{
    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame->mId.first<mLastRelocFrameId.first+params::tracking::miMaxFrames && nKFs>params::tracking::miMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3; // FIXME: 新增　20200109
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();
     // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(true) // 双目
    {
        for(int i =0; i<mCurrentFrame->N; i++)
        {
            if(mCurrentFrame->mvDepth[i]>0 && mCurrentFrame->mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame->mvpMapPoints[i] && !mCurrentFrame->mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame->mId.first>=mLastKeyFrameId.first+params::tracking::miMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame->mId.first>=mLastKeyFrameId.first+params::tracking::miMinFrames && bLocalMappingIdle);
    //Condition 1c: trackng is weak
    const bool c1c = false;//mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose);  //新增双目条件　20200109 
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 =((mnMatchesInliers<nRefMatches*params::tracking::mfThRefRatio || bNeedToInsertClose) && mnMatchesInliers>25);//params::tracking::miMatchesInliersThres);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
             if(true) //双目mSensor==System::Stereo
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
        }
    }
    else
        return false;

}


bool Tracking::NeedNewBundledKeyFrames()
{
    // if(mbOnlyTracking) viewer todo
    //     return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
 
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;
  
    const int nBKFs = mpBMap->BundledKeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame->mId.first<mLastRelocFrameId.first+params::tracking::miMaxFrames && nBKFs>params::tracking::miMaxFrames)
    {
        return false;
    }

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3; // FIXME: 新增　20200109
    if(nBKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceBKFs->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();
     // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=CentralControl::MONOCULAR) // for multi-camera
    {
        for(int i =0; i<mCurrentFrame->N_L_R; i++)
        {
            if(mCurrentFrame->mvBDepth[i]>0 && mCurrentFrame->mvBDepth[i]<mThDepth)
            {
                if(mCurrentFrame->mvpMapPointsBKFs[i] && !mCurrentFrame->mvbOutlierBKFs[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nBKFs<2)
        thRefRatio = 0.4f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame->mId.first>=mLastKeyFrameId.first+params::tracking::miMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame->mId.first>=mLastKeyFrameId.first+params::tracking::miMinFrames && bLocalMappingIdle);
    //Condition 1c: trackng is weak
    const bool c1c = mSensor!=CentralControl::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 =((mnMatchesInliers<nRefMatches*params::tracking::mfThRefRatio || bNeedToInsertClose) && mnMatchesInliers>15);//params::tracking::miMatchesInliersThres);
  
    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
             if(mSensor!=CentralControl::MONOCULAR)
            {
                if(mpLocalMapper->BundledKeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;

}

//双目需要加上建图
void Tracking::CreateNewKeyFrame()
{
    
    if(!mpLocalMapper->SetNotStop(true))
        return;

    kfptr pKF{new KeyFrame(*mCurrentFrame,mpMap,mpKeyFrameDB,mpComm,eSystemState::CLIENT,-1)};

    std::vector<mpptr> vpM = pKF->GetMapPointMatches();
    for(vector<mpptr>::const_iterator vit = vpM.begin();vit!=vpM.end();++vit)
    {
        mpptr pMPi = *vit;

        if(!pMPi)
            continue;

        if(pMPi->isBad())
            continue;

        if(pMPi->mId.second != mClientId)
        {
            pMPi->SetMultiUse();
        }
    }

    mpReferenceKF = pKF;
    mCurrentFrame->mpReferenceKF = pKF;

    if(mSensor!=CentralControl::MONOCULAR)
    {
        mCurrentFrame->UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame->N);
        for(int i=0; i<mCurrentFrame->N; i++)
        {
            float z = mCurrentFrame->mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            int bCreateNew_count=0;// zmf add

            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                mpptr pMP = mCurrentFrame->mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame->mvpMapPoints[i] = nullptr;
                }

                if(bCreateNew)
                {
                    bCreateNew_count++;//zmf add @2019.12.27

                    cv::Mat x3D = mCurrentFrame->UnprojectStereo(i);
                    //todo
                    // mpptr pNewMP{new MapPoint(x3D, pKF, mpMap,mClientId,mpComm,eSystemState::CLIENT,-1)};
                    // pNewMP->AddObservation(pKF,i);
                    // pKF->AddMapPoint(pNewMP,i);
                    // pNewMP->ComputeDistinctiveDescriptors();
                    // pNewMP->UpdateNormalAndDepth();
                    // mpMap->AddMapPoint(pNewMP);

                    // mCurrentFrame->mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mLastKeyFrameId = mCurrentFrame->mId;
    mpLastKeyFrame = pKF;
}


void Tracking::CreateNewBundledKeyFrames()
{
    
    if(!mpLocalMapper->SetNotStop(true))
        return;

    bkfptr pBKFs{new BundledKeyFrames(*mCurrentFrame, mpBMap, mpBundledKeyFramesDB,mpComm,eSystemState::CLIENT,-1)};

    std::vector<mpptr> vpM = pBKFs->GetMapPointMatches(); 
    for(vector<mpptr>::const_iterator vit = vpM.begin();vit!=vpM.end();++vit)
    {
        mpptr pMPi = *vit;

        if(!pMPi)
            continue;

        if(pMPi->isBad())
            continue;

        if(pMPi->mId.second != mClientId)
        {
            pMPi->SetMultiUse();
        }
    }

    mpReferenceBKFs = pBKFs;
    mCurrentFrame->mpReferenceBKFs = pBKFs;

    if(mSensor!=CentralControl::MONOCULAR)
    {
        mCurrentFrame->UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame->N_L_R);
        for(int i=0; i<mCurrentFrame->N_L_R; i++)
        {
            float z = mCurrentFrame->mvBDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;

            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                mpptr pMP = mCurrentFrame->mvpMapPointsBKFs[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame->mvpMapPointsBKFs[i] = nullptr;
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame->ComputeMapPointInWorldFrame(mCurrentFrame->mvx3Ds[i].mPos);
                    mpptr pNewMP{new MapPoint(x3D, pBKFs, mpBMap,mClientId,mpComm,eSystemState::CLIENT,-1)};
                    //Add mappoint's observation(KF, BKFs)
                    pNewMP->AddBKFsObservation(pBKFs, i, cameraNum);
                    //Add mappoint into KF and BKFs
                    pBKFs->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepthPlus();

                    mpBMap->AddMapPoint(pNewMP);

                    mCurrentFrame->mvpMapPointsBKFs[i]=pNewMP;
                    
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertBundledKeyFrames(pBKFs);

    mpLocalMapper->SetNotStop(false);

    mLastBundledKeyFramesId = mCurrentFrame->mId;
    mpLastBundledKeyFrames = pBKFs;
    cout<<"This is a bundledkeyframe"<<endl;
}

void Tracking::SearchLocalPoints()  // the local map 
{
    // Do not search map points already matched
    for(vector<mpptr>::iterator vit=mCurrentFrame->mvpMapPoints.begin(), vend=mCurrentFrame->mvpMapPoints.end(); vit!=vend; vit++)
    {
        mpptr pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = nullptr;
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mLastFrameSeen = mCurrentFrame->mId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    int seen = 0; 
    int bad = 0;
    int notinfrustrum = 0;

    // Project points in frame and check its visibility
    for(vector<mpptr>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        mpptr pMP = *vit;
        if(pMP->mLastFrameSeen == mCurrentFrame->mId)
        {
            ++seen;
            continue;
        }
        if(pMP->isBad())
        {
            ++bad;
            continue;
        }
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame->isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
        else ++notinfrustrum;
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame->mId.first<mLastRelocFrameId.first+2)
        {
            th=5;
        }

        matcher.SearchByProjection(*mCurrentFrame,mvpLocalMapPoints,th); //  the map points associate to current frame's keypoints 
    }
}

void Tracking::SearchLocalPointsPlus()  // the local bmap 
{
    // Do not search map points already matched
    for(vector<mpptr>::iterator vit=mCurrentFrame->mvpMapPointsBKFs.begin(), vend=mCurrentFrame->mvpMapPointsBKFs.end(); vit!=vend; vit++)
    {
        mpptr pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = nullptr;
            }
            else
            {
                if(pMP->mLastFrameSeen==mCurrentFrame->mId)   //todo detele after project final done(keyframe and bkfs are unified)
                    continue;
                else
                    pMP->IncreaseVisible();
                pMP->mLastFrameSeen = mCurrentFrame->mId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    int seen = 0; 
    int bad = 0;
    int notinfrustrum = 0;

    vector<vector<mpptr>> vpLocalMapPoint(mCurrentFrame->cameraNum, vector<mpptr>());
    // Project points in frame and check its visibility

    //todo improve accuracy 
    for(int cameraId = 0; cameraId < mCurrentFrame->cameraNum; cameraId++)
    {
        for(vector<mpptr>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
        {
            mpptr pMP = *vit;

            if(pMP->mLastFrameSeen == mCurrentFrame->mId)
            {
                ++seen;
                continue;
            }
            if(pMP->isBad())
            {
                ++bad;
                continue;
            }
            // Project (this fills MapPoint variables for matching)
            if(mCurrentFrame->isInFrustum(cameraId, pMP,0.5))
            {
                pMP->IncreaseVisible();
                nToMatch++;
                vpLocalMapPoint[cameraId].push_back(pMP);
            }
            else 
                ++notinfrustrum;
        }

        if(nToMatch>0)
        {
            ORBmatcher matcher(0.8);
            int th = 1;
            // If the camera has been relocalised recently, perform a coarser search
            if(mCurrentFrame->mId.first<mLastRelocFrameId.first+2)
            {
                th=5;
            }

            int nmatcher = matcher.SearchByProjection(cameraId,*mCurrentFrame,vpLocalMapPoint[cameraId],th); //  the map points associate to current frame's keypoints 
            //cout<<"Camara Id: "<<cameraId<<"has "<<nmatcher<<"matches"<<endl;
        }
    }
  
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();

    UpdateLocalPoints();
}

void Tracking::UpdateLocalBMap()
{
    // This is for visualization
    mpBMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalBundledKeyFrames();

    UpdateLocalPointsPlus();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<kfptr>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        kfptr pKF = *itKF;
        const vector<mpptr> vpMPs = pKF->GetMapPointMatches();

        int empty = 0;

        for(vector<mpptr>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            mpptr pMP = *itMP;
            if(!pMP)
            {
                ++empty;
                continue;
            }
            if(pMP->mTrackReferenceForFrame==mCurrentFrame->mId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mTrackReferenceForFrame=mCurrentFrame->mId;
            }
        }
    }
}

void Tracking::UpdateLocalPointsPlus()
{
    mvpLocalMapPoints.clear();

    for(vector<bkfptr>::const_iterator itBKFs=mvpLocalBundledKeyFrames.begin(), itEndBKFs=mvpLocalBundledKeyFrames.end(); itBKFs!=itEndBKFs; itBKFs++)
    {
        bkfptr pBKFs = *itBKFs;
        const vector<mpptr> vpMPs = pBKFs->GetMapPointMatches();

        //int empty = 0;

        for(vector<mpptr>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            mpptr pMP = *itMP;
            if(!pMP)
            {
                //++empty;
                continue;
            }
            if(pMP->mTrackReferenceForFrame==mCurrentFrame->mId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mTrackReferenceForFrame=mCurrentFrame->mId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()  //the local keyframes for the current frame
{
    // Each map point vote for the keyframes in which it has been observed
    map<kfptr,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame->N; i++)
    {
        if(mCurrentFrame->mvpMapPoints[i])
        {
            mpptr pMP = mCurrentFrame->mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<kfptr,size_t> observations = pMP->GetObservations();
                for(map<kfptr,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++; //count for the number of observated keyframe
            }
            else
            {
                mCurrentFrame->mvpMapPoints[i]=nullptr;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    kfptr pKFmax= nullptr;

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<kfptr,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        kfptr pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;   
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mTrackReferenceForFrame = mCurrentFrame->mId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<kfptr>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        kfptr pKF = *itKF;

        const vector<kfptr> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<kfptr>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            kfptr pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mTrackReferenceForFrame!=mCurrentFrame->mId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mTrackReferenceForFrame=mCurrentFrame->mId;
                    break;
                }
            }
        }

        const set<kfptr> spChilds = pKF->GetChilds();
        for(set<kfptr>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            kfptr pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mTrackReferenceForFrame!=mCurrentFrame->mId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mTrackReferenceForFrame=mCurrentFrame->mId;
                    break;
                }
            }
        }

        kfptr pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mTrackReferenceForFrame!=mCurrentFrame->mId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mTrackReferenceForFrame=mCurrentFrame->mId;
                break;
            }
        }
    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame->mpReferenceKF = mpReferenceKF;
    }
}

void Tracking::UpdateLocalBundledKeyFrames()  //the local bundled keyframes for the current frame
{
    // Each map point vote for the keyframes in which it has been observed
    map<bkfptr,int> bundledkeyframeCounter;
    for(int i=0; i<mCurrentFrame->N_L_R; i++)
    {
        if(mCurrentFrame->mvpMapPointsBKFs[i])
        {
            mpptr pMP = mCurrentFrame->mvpMapPointsBKFs[i];
            if(!pMP->isBad())
            {
                const map<bkfptr,size_t> observations = pMP->GetBKFsObservations();
                for(map<bkfptr,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    bundledkeyframeCounter[it->first]++; //count for the number of observated keyframe
            }
            else
            {
                mCurrentFrame->mvpMapPointsBKFs[i]=nullptr;
            }
        }
    }

    if(bundledkeyframeCounter.empty())
        return;

    int max=0;
    bkfptr pBKFsmax= nullptr;

    mvpLocalBundledKeyFrames.clear();
    mvpLocalBundledKeyFrames.reserve(3*bundledkeyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<bkfptr,int>::const_iterator it=bundledkeyframeCounter.begin(), itEnd=bundledkeyframeCounter.end(); it!=itEnd; it++)
    {
        bkfptr pBKFs = it->first;

        if(pBKFs->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pBKFsmax=pBKFs;   
        }

        mvpLocalBundledKeyFrames.push_back(it->first);
        pBKFs->mTrackReferenceForFrame = mCurrentFrame->mId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<bkfptr>::const_iterator itBKFs=mvpLocalBundledKeyFrames.begin(), itEndBKFs=mvpLocalBundledKeyFrames.end(); itBKFs!=itEndBKFs; itBKFs++)
    {
        // Limit the number of keyframes
        if(mvpLocalBundledKeyFrames.size()>80)
            break;

        bkfptr pBKFs = *itBKFs;

        const vector<bkfptr> vNeighs = pBKFs->GetBestCovisibilityBundledKeyFrames(10);

        for(vector<bkfptr>::const_iterator itNeighBKFs=vNeighs.begin(), itEndNeighBKFs=vNeighs.end(); itNeighBKFs!=itEndNeighBKFs; itNeighBKFs++)
        {
            bkfptr pNeighBKFs = *itNeighBKFs;
            if(!pNeighBKFs->isBad())
            {
                if(pNeighBKFs->mTrackReferenceForFrame!=mCurrentFrame->mId)
                {
                    mvpLocalBundledKeyFrames.push_back(pNeighBKFs);
                    pNeighBKFs->mTrackReferenceForFrame=mCurrentFrame->mId;
                    break;
                }
            }
        }

        const set<bkfptr> spChilds = pBKFs->GetChilds();
        for(set<bkfptr>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            bkfptr pChildBKFs = *sit;
            if(!pChildBKFs->isBad())
            {
                if(pChildBKFs->mTrackReferenceForFrame!=mCurrentFrame->mId)
                {
                    mvpLocalBundledKeyFrames.push_back(pChildBKFs);
                    pChildBKFs->mTrackReferenceForFrame=mCurrentFrame->mId;
                    break;
                }
            }
        }

        bkfptr pParent = pBKFs->GetParent();
        if(pParent)
        {
            if(pParent->mTrackReferenceForFrame!=mCurrentFrame->mId)
            {
                mvpLocalBundledKeyFrames.push_back(pParent);
                pParent->mTrackReferenceForFrame=mCurrentFrame->mId;
                break;
            }
        }
    }

    if(pBKFsmax)
    {
        mpReferenceBKFs = pBKFsmax;
        mCurrentFrame->mpReferenceBKFs = mpReferenceBKFs;
    }
}


void Tracking::Reset()
{
    cout << "    System Reseting" << endl;
    mpViewer->RequestReset();
    mpLocalMapper->RequestReset();
    mpKeyFrameDB->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;
    MapPoint::nNextId = 0;

    if(mpInitializer)
        mpInitializer = nullptr;

    mpComm->RequestReset();

    mpMap->clear();

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    cout << "    Reseting Done..." << endl;
}

Tracking::kfptr Tracking::GetReferenceKF()
{
    //Works w/o mutex, since tracking is locked when this method is called by comm

    if(!mpCC->IsTrackingLocked())
        cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " Tracking assumed to be locked " << endl;

    return mpReferenceKF;
}

Tracking::bkfptr Tracking::GetReferenceBKFs()
{
    //Works w/o mutex, since tracking is locked when this method is called by comm

    if(!mpCC->IsTrackingLocked())
        cout << "\033[1;33m!!! WARN !!!\033[0m " << __func__ << ":" << __LINE__ << " Tracking assumed to be locked " << endl;

    return mpReferenceBKFs;
}

} //end ns
