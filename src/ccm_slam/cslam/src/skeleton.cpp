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

#include <cslam/skeleton.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include<chrono>
#include<random>
#include<set>
#include<cmath>

 
using namespace cv;
using namespace cv::dnn;
using namespace std;
namespace cslam {
    /////////////////////////////////////////////////////////////人体骨架识别代码
    struct KeyPoint{
	KeyPoint(cv::Point point,float probability){
		this->id = -1;
		this->point = point;
		this->probability = probability;
	}

	int id;
	cv::Point point;
	float probability;
    };

    std::ostream& operator << (std::ostream& os, const KeyPoint& kp)
    {
        os << "Id:" << kp.id << ", Point:" << kp.point << ", Prob:" << kp.probability << std::endl;
        return os;
    }
    ////////////////////////////////

    struct ValidPair{
        ValidPair(int aId,int bId,float score){
            this->aId = aId;
            this->bId = bId;
            this->score = score;
        }

        int aId;
        int bId;
        float score;
    };

    std::ostream& operator << (std::ostream& os, const ValidPair& vp)
    {
        os << "A:" << vp.aId << ", B:" << vp.bId << ", score:" << vp.score << std::endl;
        return os;
    }
    ////////////////////////////////

    template < class T > std::ostream& operator << (std::ostream& os, const std::vector<T>& v)
    {
        os << "[";
        bool first = true;
        for (typename std::vector<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii, first = false)
        {
            if(!first) os << ",";
            os << " " << *ii;
        }
        os << "]";
        return os;
    }

    template < class T > std::ostream& operator << (std::ostream& os, const std::set<T>& v)
    {
        os << "[";
        bool first = true;
        for (typename std::set<T>::const_iterator ii = v.begin(); ii != v.end(); ++ii, first = false)
        {
            if(!first) os << ",";
            os << " " << *ii;
        }
        os << "]";
        return os;
    }
    const int nPoints = 18;

    const std::string keypointsMapping[] = {
        "Nose", "Neck",
        "R-Sho", "R-Elb", "R-Wr",
        "L-Sho", "L-Elb", "L-Wr",
        "R-Hip", "R-Knee", "R-Ank",
        "L-Hip", "L-Knee", "L-Ank",
        "R-Eye", "L-Eye", "R-Ear", "L-Ear"
    };


    const std::vector<std::pair<int,int>> mapIdx = {
        {31,32}, {39,40}, {33,34}, {35,36}, {41,42}, {43,44},
        {19,20}, {21,22}, {23,24}, {25,26}, {27,28}, {29,30},
        {47,48}, {49,50}, {53,54}, {51,52}, {55,56}, {37,38},
        {45,46}
    };

    const std::vector<std::pair<int,int>> posePairs = {
        {1,2}, {1,5}, {2,3}, {3,4}, {5,6}, {6,7},
        {1,8}, {8,9}, {9,10}, {1,11}, {11,12}, {12,13},
        {1,0}, {0,14}, {14,16}, {0,15}, {15,17}, {2,17},
        {5,16}
    };

    void getKeyPoints(cv::Mat& probMap,double threshold,std::vector<KeyPoint>& keyPoints){
        cv::Mat smoothProbMap;
        cv::GaussianBlur( probMap, smoothProbMap, cv::Size( 3, 3 ), 0, 0 );
        cv::Mat maskedProbMap;
        cv::threshold(smoothProbMap,maskedProbMap,threshold,255,cv::THRESH_BINARY);
        maskedProbMap.convertTo(maskedProbMap,CV_8U,1);
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(maskedProbMap,contours,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
        for(int i = 0; i < contours.size();++i){
            cv::Mat blobMask = cv::Mat::zeros(smoothProbMap.rows,smoothProbMap.cols,smoothProbMap.type());
            cv::fillConvexPoly(blobMask,contours[i],cv::Scalar(1));
            double maxVal;
            cv::Point maxLoc;
            cv::minMaxLoc(smoothProbMap.mul(blobMask),0,&maxVal,0,&maxLoc);
            keyPoints.push_back(KeyPoint(maxLoc, probMap.at<float>(maxLoc.y,maxLoc.x)));
        }
    }

    void populateColorPalette(std::vector<cv::Scalar>& colors,int nColors){
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis1(64, 200);
        std::uniform_int_distribution<> dis2(100, 255);
        std::uniform_int_distribution<> dis3(100, 255);
        for(int i = 0; i < nColors;++i){
            colors.push_back(cv::Scalar(dis1(gen),dis2(gen),dis3(gen)));
        }
    }

    void splitNetOutputBlobToParts(cv::Mat& netOutputBlob,const cv::Size& targetSize,std::vector<cv::Mat>& netOutputParts){
        int nParts = netOutputBlob.size[1];
        int h = netOutputBlob.size[2];
        int w = netOutputBlob.size[3];
        for(int i = 0; i< nParts;++i){
            cv::Mat part(h, w, CV_32F, netOutputBlob.ptr(0,i));
            cv::Mat resizedPart;
            cv::resize(part,resizedPart,targetSize);
            netOutputParts.push_back(resizedPart);
        }
    }

    void populateInterpPoints(const cv::Point& a,const cv::Point& b,int numPoints,std::vector<cv::Point>& interpCoords){
        float xStep = ((float)(b.x - a.x))/(float)(numPoints-1);
        float yStep = ((float)(b.y - a.y))/(float)(numPoints-1);
        interpCoords.push_back(a);
        for(int i = 1; i< numPoints-1;++i){
            interpCoords.push_back(cv::Point(a.x + xStep*i,a.y + yStep*i));
        }
        interpCoords.push_back(b);
    }

    void getValidPairs(const std::vector<cv::Mat>& netOutputParts,
                    const std::vector<std::vector<KeyPoint>>& detectedKeypoints,
                    std::vector<std::vector<ValidPair>>& validPairs,
                    std::set<int>& invalidPairs) {

        int nInterpSamples = 10;
        float pafScoreTh = 0.1;
        float confTh = 0.7;

        for(int k = 0; k < mapIdx.size();++k ){
            //A->B constitute a limb
            cv::Mat pafA = netOutputParts[mapIdx[k].first];
            cv::Mat pafB = netOutputParts[mapIdx[k].second];
            //Find the keypoints for the first and second limb
            const std::vector<KeyPoint>& candA = detectedKeypoints[posePairs[k].first];
            const std::vector<KeyPoint>& candB = detectedKeypoints[posePairs[k].second];
            int nA = candA.size();
            int nB = candB.size();
            /*
            # If keypoints for the joint-pair is detected
            # check every joint in candA with every joint in candB
            # Calculate the distance vector between the two joints
            # Find the PAF values at a set of interpolated points between the joints
            # Use the above formula to compute a score to mark the connection valid
            */

            if(nA != 0 && nB != 0){
                std::vector<ValidPair> localValidPairs;
                for(int i = 0; i< nA;++i){
                    int maxJ = -1;
                    float maxScore = -1;
                    bool found = false;
                    for(int j = 0; j < nB;++j){
                        std::pair<float,float> distance(candB[j].point.x - candA[i].point.x,candB[j].point.y - candA[i].point.y);
                        float norm = std::sqrt(distance.first*distance.first + distance.second*distance.second);
                        if(!norm){
                            continue;
                        }
                        distance.first /= norm;
                        distance.second /= norm;
                        //Find p(u)
                        std::vector<cv::Point> interpCoords;
                        populateInterpPoints(candA[i].point,candB[j].point,nInterpSamples,interpCoords);
                        //Find L(p(u))
                        std::vector<std::pair<float,float>> pafInterp;
                        for(int l = 0; l < interpCoords.size();++l){
                            pafInterp.push_back(
                                std::pair<float,float>(
                                    pafA.at<float>(interpCoords[l].y,interpCoords[l].x),
                                    pafB.at<float>(interpCoords[l].y,interpCoords[l].x)
                                ));
                        }
                        std::vector<float> pafScores;
                        float sumOfPafScores = 0;
                        int numOverTh = 0;
                        for(int l = 0; l< pafInterp.size();++l){
                            float score = pafInterp[l].first*distance.first + pafInterp[l].second*distance.second;
                            sumOfPafScores += score;
                            if(score > pafScoreTh){
                                ++numOverTh;
                            }
                            pafScores.push_back(score);
                        }
                        float avgPafScore = sumOfPafScores/((float)pafInterp.size());
                        if(((float)numOverTh)/((float)nInterpSamples) > confTh){
                            if(avgPafScore > maxScore){
                                maxJ = j;
                                maxScore = avgPafScore;
                                found = true;
                            }
                        }
                    }/* j */

                    if(found){
                        localValidPairs.push_back(ValidPair(candA[i].id,candB[maxJ].id,maxScore));
                    }
                }/* i */

                validPairs.push_back(localValidPairs);

            } else {
                invalidPairs.insert(k);
                validPairs.push_back(std::vector<ValidPair>());
            }
        }/* k */
    }

    void getPersonwiseKeypoints(const std::vector<std::vector<ValidPair>>& validPairs,
                                const std::set<int>& invalidPairs,
                                std::vector<std::vector<int>>& personwiseKeypoints) {
        for(int k = 0; k < mapIdx.size();++k){
            if(invalidPairs.find(k) != invalidPairs.end()){
                continue;
            }
            const std::vector<ValidPair>& localValidPairs(validPairs[k]);
            int indexA(posePairs[k].first);
            int indexB(posePairs[k].second);
            for(int i = 0; i< localValidPairs.size();++i){
                bool found = false;
                int personIdx = -1;
                for(int j = 0; !found && j < personwiseKeypoints.size();++j){
                    if(indexA < personwiseKeypoints[j].size() &&
                    personwiseKeypoints[j][indexA] == localValidPairs[i].aId){
                        personIdx = j;
                        found = true;
                    }
                }/* j */

                if(found){
                    personwiseKeypoints[personIdx].at(indexB) = localValidPairs[i].bId;
                } else if(k < 17){
                    std::vector<int> lpkp(std::vector<int>(18,-1));
                    lpkp.at(indexA) = localValidPairs[i].aId;
                    lpkp.at(indexB) = localValidPairs[i].bId;
                    personwiseKeypoints.push_back(lpkp);
                }
            }/* i */
        }/* k */
    }
    /////////////////////////////////////////////////////////////////////////////
    vector<Vec3b> readColors() 
    {
        String labelFile = "/home/zmf/ccmslam_ws/model/pascal-classes.txt";
        vector<Vec3b> colors;
        // ifstream fp(labelFile);
        ifstream fp;
        fp.open(labelFile.c_str());
        if (!fp.is_open()) {
            printf("could not open the file...\n");
            exit(-1);
        }
        string line;
        while (!fp.eof()) {
            getline(fp, line);
            if (line.length()) {
                stringstream ss(line);
                string name;
                ss >> name;
                int temp;
                Vec3b color;
                ss >> temp;
                color[0] = (uchar)temp;
                ss >> temp;
                color[1] = (uchar)temp;
                ss >> temp;
                color[2] = (uchar)temp;
                colors.push_back(color);
            }
        }
        return colors;
    }

    Skeleton::Skeleton()//还未确定构造函数参数
    {
        
        ///////////////////////////////////////////////////////////////////////////////////////图像分割模型参数  暂时注释
        // String modelFile = "/home/zmf/ccmslam_ws/model/fcn8s-heavy-pascal.caffemodel";
        // String model_text_file = "/home/zmf/ccmslam_ws/model/fcn8s-heavy-pascal.prototxt";
        // net = readNetFromCaffe(model_text_file, modelFile);
        // cout<<"网络初始化完成..."<<endl;
        //////////////////////////////////////////////////////////////////////////////////////图像分割模型参数  暂时注释
        
        
    }
    void Skeleton::img_write(const double &_timestamp,const cv::Mat &im,const cv::Mat &_depth_img)//本函数负责将tracking 线程那边的图片传过来
    {
        unique_lock<mutex> lock(mMutexNewskeleton);//图像和时间错写进来加锁
        if(!im.empty()&&!_depth_img.empty())//左目图像和深度图像都不为空 
        {
            Skeleton_Frame sl;
            sl.timestamp =_timestamp;
            sl. skeleton_img=im;
            sl.depth_img=_depth_img;
            if(mlNewSkeletons.size() >3)
            {
                mlNewSkeletons.pop_front();
               
            }
            mlNewSkeletons.push_back(sl);
    
        }
            
        else return;
    }
    int Skeleton::image_segmentation()//图像分割处理函数
    {
        const size_t width = 300;
        const size_t height = 300;

        Mat frame = skeleton_img;
        
        //Mat frame = imread("/home/zmf/1.png");
        if (frame.empty()) {
            printf("could not load image...\n");
            return -1;
        }
        cv::cvtColor(frame, frame, COLOR_GRAY2BGR);
        //cvtColor(frame,frame,CV_BGR2GRAY);
        //imshow("图片测试",frame);
        //waitKey(25);
        cout<<frame.cols<<" "<<frame.rows<<" "<<frame.channels()<<endl;
        //
        //cout<<"图片通道数目="<<frame.channels()<<endl;
        //cout<<"图片读取完成..."<<endl;
        namedWindow("input image", CV_WINDOW_AUTOSIZE);
        imshow("input image", frame);
        resize(frame, frame, Size(500, 500));
        vector<Vec3b> colors = readColors();
    
        // init net
        //cout<<"图片预处理..."<<endl;
        Mat blobImage = blobFromImage(frame);
        //cout<<"预处理完成..."<<endl;
    
        // use net
        float time = getTickCount();
        net.setInput(blobImage, "data");
        //cout<<"网络前向传播..."<<endl;
        Mat score = net.forward("score");
        //Mat score = net.forward();
        //cout<<"前向传播完成..."<<endl;
        float tt = getTickCount() - time;
        printf("time consume: %.2f \n", (tt / getTickFrequency()) * 1000);
    
        // segmentation and display
        const int rows = score.size[2];
        const int cols = score.size[3];
        const int chns = score.size[1];
        Mat maxCl(rows, cols, CV_8UC1);
        Mat maxVal(rows, cols, CV_32FC1);
    
        // setup LUT
        for (int c = 0; c < chns; c++) {
            for (int row = 0; row < rows; row++) {
                const float *ptrScore = score.ptr<float>(0, c, row);
                uchar *ptrMaxCl = maxCl.ptr<uchar>(row);
                float *ptrMaxVal = maxVal.ptr<float>(row);
                for (int col = 0; col < cols; col++) {
                    if (ptrScore[col] > ptrMaxVal[col]) {
                        ptrMaxVal[col] = ptrScore[col];
                        ptrMaxCl[col] = (uchar)c;
                    }
                }
            }
        }
    
        // look up colors
        Mat result = Mat::zeros(rows, cols, CV_8UC3);
        for (int row = 0; row < rows; row++) {
            const uchar *ptrMaxCl = maxCl.ptr<uchar>(row);
            Vec3b *ptrColor = result.ptr<Vec3b>(row);
            for (int col = 0; col < cols; col++) {
                ptrColor[col] = colors[ptrMaxCl[col]];
            }
        }
        Mat dst;
        addWeighted(frame, 0.3, result, 0.7, 0, dst);
        //imwrite("FCN.jpg",dst);
        imshow("FCN-demo", dst);
        cout<<"显示网络处理结果..."<<endl; 	
        waitKey(25);
        return 0;
        
    }
    int Skeleton::skeleton_process()
    {
        {
            unique_lock<mutex> lock(mMutexNewskeleton);
            if(!mlNewSkeletons.empty())
            {
                auto tmpFrame = mlNewSkeletons.front();
                timestamp=tmpFrame.timestamp;//图像时间戳
                skeleton_img=tmpFrame.skeleton_img;//骨架图像
                depth_img=tmpFrame.depth_img;//深度图像
                mlNewSkeletons.pop_front();

            }
            
        }
        skepos.clear();
        cv::Mat input;
        skeleton_img.copyTo(input);
        if (input.empty()) {
            printf("could not load image...\n");
            return -1;
        }
        cv::cvtColor(input, input, COLOR_GRAY2BGR);
        std::chrono::time_point<std::chrono::system_clock> startTP = std::chrono::system_clock::now();
        cv::dnn::Net inputNet = cv::dnn::readNetFromCaffe("/home/zmf/models/pose_deploy_linevec.prototxt","/home/zmf/models/pose_iter_440000.caffemodel");
        cv::Mat inputBlob = cv::dnn::blobFromImage(input,1.0/255.0,cv::Size((int)((368*input.cols)/input.rows),368),cv::Scalar(0,0,0),false,false);
        inputNet.setInput(inputBlob);
        cv::Mat netOutputBlob = inputNet.forward();
        std::vector<cv::Mat> netOutputParts;
        splitNetOutputBlobToParts(netOutputBlob,cv::Size(input.cols,input.rows),netOutputParts);
        std::chrono::time_point<std::chrono::system_clock> finishTP = std::chrono::system_clock::now();
        std::cout << "Time Taken in forward pass = " << std::chrono::duration_cast<std::chrono::milliseconds>(finishTP - startTP).count() << " ms" << std::endl;
        int keyPointId = 0;
        std::vector<std::vector<KeyPoint>> detectedKeypoints;
        std::vector<KeyPoint> keyPointsList;

        for(int i = 0; i < nPoints;++i){
            std::vector<KeyPoint> keyPoints;
            getKeyPoints(netOutputParts[i],0.1,keyPoints);
            std::cout << "Keypoints - " << keypointsMapping[i] << " : " << keyPoints << std::endl;
            ///或者18个点在这边输出

            for(int i = 0; i< keyPoints.size();++i,++keyPointId){
                keyPoints[i].id = keyPointId;
            }
            detectedKeypoints.push_back(keyPoints);
            keyPointsList.insert(keyPointsList.end(),keyPoints.begin(),keyPoints.end());
        }
        std::vector<cv::Scalar> colors;
        std::vector<float> skepos_temp(3);
        populateColorPalette(colors,nPoints);
        cv::Mat outputFrame;
        input.copyTo(outputFrame);
        for(int i = 0; i < nPoints;++i){
            for(int j = 0; j < detectedKeypoints[i].size();++j){
                cv::circle(outputFrame,detectedKeypoints[i][j].point,5,colors[i],-1,cv::LINE_AA);
                //这里负责取出骨架坐标信息 zmf add
                skepos_temp[0]=detectedKeypoints[i][j].point.x;
                skepos_temp[1]=detectedKeypoints[i][j].point.y;
                skepos_temp[2]=(float)depth_img.at<uchar>(skepos_temp[0],skepos_temp[1]);
                skepos.push_back(skepos_temp[0]);
                skepos.push_back(skepos_temp[1]);
                skepos.push_back(skepos_temp[2]);
            }
        }
        std::vector<std::vector<ValidPair>> validPairs;
        std::set<int> invalidPairs;
        getValidPairs(netOutputParts,detectedKeypoints,validPairs,invalidPairs);
        std::vector<std::vector<int>> personwiseKeypoints;
        getPersonwiseKeypoints(validPairs,invalidPairs,personwiseKeypoints);

        for(int i = 0; i< nPoints-1;++i){
            for(int n  = 0; n < personwiseKeypoints.size();++n){
                const std::pair<int,int>& posePair = posePairs[i];
                int indexA = personwiseKeypoints[n][posePair.first];
                int indexB = personwiseKeypoints[n][posePair.second];

                if(indexA == -1 || indexB == -1){
                    continue;
                }
                const KeyPoint& kpA = keyPointsList[indexA];
                const KeyPoint& kpB = keyPointsList[indexB];
                cv::line(outputFrame,kpA.point,kpB.point,colors[i],3,cv::LINE_AA);
            }
        }
        // push SkPos 
        cv::imshow("Detected Pose",outputFrame);
        cv::waitKey(25);
        
        return 0;
    }
    void Skeleton::RunClient()
    {
        
        while(1)
        {
            //cout<<"Skeleton 线程运行中..."<<endl;
            if(skeleton_process()==0)
            {
                if(this->skepos.size()>0)
                {//提取的骨架点数目大于0才传给comm
                    mpComm->PassSkPostoComm(this->shared_from_this());//算完之后发布骨架信息到comm
                }
                //mpComm->PassSkPostoComm(this->shared_from_this());//算完之后发布骨架信息到comm
            }
        }
    }
    

  

   

} //end ns