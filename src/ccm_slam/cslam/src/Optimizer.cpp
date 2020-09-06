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

#include <cslam/Optimizer.h>

namespace cslam {

void Optimizer::GlobalBundleAdjustemntClient(mapptr pMap, size_t ClientId, int nIterations, bool* pbStopFlag, const idpair nLoopKF, const bool bRobust)
{
    vector<kfptr> vpKFs = pMap->GetAllKeyFrames();
    vector<mpptr> vpMP = pMap->GetAllMapPoints();
    BundleAdjustmentClient(vpKFs,vpMP,ClientId,nIterations,pbStopFlag, nLoopKF, bRobust);
}


void Optimizer::BundleAdjustmentClient(const vector<kfptr> &vpKFs, const vector<mpptr> &vpMP, size_t ClientId,
                                 int nIterations, bool* pbStopFlag, const idpair nLoopKF, const bool bRobust)
{
    const idpair zeropair = make_pair(0,ClientId);

    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    // Set KeyFrame vertices
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        kfptr pKF = vpKFs[i];
        if(pKF->isBad())
            continue;

        if(pKF->mId.first >= IDRANGE)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
            throw infrastructure_ex();
        }

        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(Optimizer::GetID(pKF->mId,true));
        vSE3->setFixed(pKF->mId == zeropair);
        optimizer.addVertex(vSE3);
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815); //双目需要用到的

    // Set MapPoint vertices
    for(size_t i=0; i<vpMP.size(); i++)
    {
        mpptr pMP = vpMP[i];
        if(pMP->isBad())
            continue;

        if(pMP->mId.first >= IDRANGE)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": MP index out of bounds" << endl;
            throw infrastructure_ex();
        }

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = Optimizer::GetID(pMP->mId,false);
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<kfptr,size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for(map<kfptr,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            kfptr pKF = mit->first;
            if(pKF->isBad())
                continue;

            if(pKF->mId.first >= IDRANGE)
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
                throw infrastructure_ex();
            }

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];


            if(pKF->mvuRight[mit->second]<0)//单目
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(Optimizer::GetID(pKF->mId,true))));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            }
            else //双目部分，需要添加部分
            {
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(Optimizer::GetID(pKF->mId,true))));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e); 
            }
            
        }

        if(nEdges==0)
        {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i]=true;
        }
        else
        {
            vbNotIncludedMP[i]=false;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        kfptr pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(Optimizer::GetID(pKF->mId,true)));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if(nLoopKF==zeropair)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat),false);
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            //Add
            Mat mTcwGBA_tmp = Converter::toCvMat(SE3quat)*pKF->GetPoseInverse();
            Eigen::Matrix4d testGBA = Converter::toMatrix4d(mTcwGBA_tmp);
            Eigen::Matrix4d poseBef = Converter::toMatrix4d(pKF->GetPoseInverse());
            Eigen::Matrix4d poseAfGBA = Converter::toMatrix4d(Converter::toCvMat(SE3quat).inv());

            if(testGBA.block<3,1>(0,3).norm()>0.5)
            {
                cout<<"第"<<pKF->mId.second<<" client, 第"<<pKF->mId.first<<" client"<<"时间戳:"<<setprecision(25)<<pKF->mTimeStamp<<"的GBA(client)效果不好!!!!"<<endl;
                cout<<"GBA(CLIENT)之前的位置： "<<poseBef.block<3,1>(0,3).transpose()<<endl;
                cout<<"GBA(CLIENT)之后的位置： "<<poseAfGBA.block<3,1>(0,3).transpose()<<endl;
            }
            //End
            pKF->mBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        mpptr pMP = vpMP[i];

        if(pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(Optimizer::GetID(pMP->mId,false)));

        if(nLoopKF==zeropair)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()),false);
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mBAGlobalForKF = nLoopKF;
        }
    }
}


int Optimizer::PoseOptimizationClient(Frame &Frame)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    // Set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(Frame.mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);

    // Set MapPoint vertices
    const int N = Frame.N;
    //for the mono
    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    //加入双目优化用的点和边
    vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
    vector<size_t> vnIndexEdgeStereo;
    vpEdgesStereo.reserve(N);
    vnIndexEdgeStereo.reserve(N);

    const float deltaMono = sqrt(5.991);
    const float deltaStereo = sqrt(7.815); // 添加双目的优化参数

    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for(int i=0; i<N; i++)
        {
            mpptr pMP = Frame.mvpMapPoints[i];
            if(pMP)
            {
                if(Frame.mvuRight[i] < 0)
                {
                    nInitialCorrespondences++;
                    Frame.mvbOutlier[i] = false;

                    Eigen::Matrix<double,2,1> obs;
                    const cv::KeyPoint &kpUn = Frame.mvKeysUn[i];
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = Frame.mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->fx = Frame.fx;
                    e->fy = Frame.fy;
                    e->cx = Frame.cx;
                    e->cy = Frame.cy;
                    cv::Mat Xw = pMP->GetWorldPos();
                    e->Xw[0] = Xw.at<float>(0);
                    e->Xw[1] = Xw.at<float>(1);
                    e->Xw[2] = Xw.at<float>(2);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                }
                else
                {
                    nInitialCorrespondences++;
                    Frame.mvbOutlier[i] = false;

                    //SET EDGE
                    Eigen::Matrix<double,3,1> obs;// 这里和单目不同
                    const cv::KeyPoint &kpUn = Frame.mvKeysUn[i];
                    const float &kp_ur = Frame.mvuRight[i];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;// 这里和单目不同

                    g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();// 这里和单目不同

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = Frame.mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaStereo);

                    e->fx = Frame.fx;
                    e->fy = Frame.fy;
                    e->cx = Frame.cx;
                    e->cy = Frame.cy;
                    e->bf = Frame.mbf;
                    cv::Mat Xw = pMP->GetWorldPos();
                    e->Xw[0] = Xw.at<float>(0);
                    e->Xw[1] = Xw.at<float>(1);
                    e->Xw[2] = Xw.at<float>(2);

                    optimizer.addEdge(e);

                    vpEdgesStereo.push_back(e);
                    vnIndexEdgeStereo.push_back(i);                    
                }
            }
        }
    }


    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const float chi2Stereo[4]={7.815,7.815,7.815, 7.815}; //双目的卡方检验参数
    const int its[4]={10,10,10,10};

    int nBad=0;

    for(size_t it=0; it<1; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(Frame.mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if(Frame.mvbOutlier[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                Frame.mvbOutlier[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                Frame.mvbOutlier[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        //接着对双目的边进行优化
        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];
            
            const size_t idx = vnIndexEdgeStereo[i];
            if(Frame.mvbOutlier[idx])
            {
                e->computeError();
            }
            const float chi2 = e->chi2();

            if(chi2 > chi2Stereo[it])
            {
                Frame.mvbOutlier[idx] = true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                e->setLevel(0);
                Frame.mvbOutlier[idx]= false;
            }

            if(it ==2 )
                e->setRobustKernel(0);
        }
        if(optimizer.edges().size()<10)
            break;
    }

    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    Frame.SetPose(pose);

    return nInitialCorrespondences-nBad;
}

int Optimizer::PoseOptimizationClientPlus(Frame &Frame)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences=0;

    //set Frame vertex
    g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
    vSE3->setEstimate(Converter::toSE3Quat(Frame.mTcw));
    vSE3->setId(0);
    vSE3->setFixed(false);
    optimizer.addVertex(vSE3);


    //set MapPoint vertices
    const int N = Frame.N_L_R;

    vector<g2o::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono0;
    vector<size_t> vnIndexEdgeMono0;
    vpEdgesMono0.reserve(N);
    vnIndexEdgeMono0.reserve(N);

    vector<g2o::EdgeSE3ProjectXYZOnlyPosePlus*> vpEdgesMono1;
    vector<size_t> vnIndexEdgeMono1;
    vpEdgesMono1.reserve(N);
    vnIndexEdgeMono1.reserve(N);

    int mpcount = 0;
    const float deltaMono = sqrt(5.991);

    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for(int i=0; i<N; i++)
        {
            mpptr pMP = Frame.mvpMapPointsBKFs[i];
            if(pMP)
            {
                ++mpcount;
                vector<int> pair_index = Frame.vKeyPointsIndexMapPlus[i];
                if(pair_index[0] >= 0)//id 0 camera
                {
                    nInitialCorrespondences++;
                    Frame.mvbOutlierBKFs[i] = false;

                    Eigen::Matrix<double,2,1> obs;

                    const cv::KeyPoint &kpUn = Frame.mvKeysMultipleUn[0][pair_index[0]];
                    obs << kpUn.pt.x, kpUn.pt.y;
                    assert(pair_index[0]<Frame.mvpkeyPointsNum[0]);
                    g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = Frame.mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->fx = Frame.vfx[0];
                    e->fy = Frame.vfy[0];
                    e->cx = Frame.vcx[0];
                    e->cy = Frame.vcy[0];
                    cv::Mat Xw = pMP->GetWorldPos();

                    e->Xw[0] = Xw.at<float>(0);
                    e->Xw[1] = Xw.at<float>(1);
                    e->Xw[2] = Xw.at<float>(2);

                    optimizer.addEdge(e);

                    vpEdgesMono0.push_back(e);
                    vnIndexEdgeMono0.push_back(i);

                }//id 1 camera
                if(pair_index[1] >= 0)
                {
                    nInitialCorrespondences++;
                    Frame.mvbOutlierBKFs[i] = false;

                    Eigen::Matrix<double,2,1> obs;

                    const cv::KeyPoint &kpUn = Frame.mvKeysMultipleUn[1][pair_index[1]];
                    obs << kpUn.pt.x, kpUn.pt.y;
                    assert(pair_index[1]<Frame.mvpkeyPointsNum[1]);
                    g2o::EdgeSE3ProjectXYZOnlyPosePlus* e = new g2o::EdgeSE3ProjectXYZOnlyPosePlus();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                    e->setMeasurement(obs);
                    const float invSigma2 = Frame.mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->fx = Frame.vfx[1];
                    e->fy = Frame.vfy[1];
                    e->cx = Frame.vcx[1];
                    e->cy = Frame.vcy[1];
                    cv::Mat Xw = pMP->GetWorldPos();
                    e->Xw[0] = Xw.at<float>(0);
                    e->Xw[1] = Xw.at<float>(1);
                    e->Xw[2] = Xw.at<float>(2);
                    cv::Mat tTcamji= cv::Mat::eye(4,4,CV_32F); //T10
                    for(int x = 0; x < 1; x++)
                    {
                        tTcamji = Frame.mvTcamji[x]* tTcamji;  //Ti0
                    }

                    e->Trl = Converter::toMatrix4d(tTcamji);
                    optimizer.addEdge(e);
                    vpEdgesMono1.push_back(e);
                    vnIndexEdgeMono1.push_back(i);
                }//id 2 camera
//                if(pair_index[2] >= 0)
//                {
//                    nInitialCorrespondences++;
//                    pFrame->mvbOutlierBKFs[i] = false;
//
//                    Eigen::Matrix<double,2,1> obs;
//
//                    const cv::KeyPoint &kpUn = pFrame->mvKeysMultipleUn[2][pair_index[2]];
//                    obs << kpUn.pt.x, kpUn.pt.y;
//
//                    g2o::EdgeSE3ProjectXYZOnlyPosePlus* e = new g2o::EdgeSE3ProjectXYZOnlyPosePlus();
//                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
//                    e->setMeasurement(obs);
//                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
//                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
//
//                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//                    e->setRobustKernel(rk);
//                    rk->setDelta(deltaMono);
//
//                    e->fx = pFrame->vfx[2];
//                    e->fy = pFrame->vfy[2];
//                    e->cx = pFrame->vcx[2];
//                    e->cy = pFrame->vcy[2];
//                    cv::Mat Xw = pMP->GetWorldPos();
//                    e->Xw[0] = Xw.at<float>(0);
//                    e->Xw[1] = Xw.at<float>(1);
//                    e->Xw[2] = Xw.at<float>(2);
//
//                    cv::Mat tTcamji= cv::Mat::eye(4,4,CV_32F); //T20
//                    for(int x = 0; x < 2; x++)
//                    {
//                        tTcamji = pFrame->mvTcamji[x]* tTcamji;
//                    }
//                    e->Trl = Converter::toMatrix4d(tTcamji);
//                    optimizer.addEdge(e);
//                    vpEdgesMonoRight.push_back(e);
//                    vnIndexEdgeMonoRight.push_back(i);
//                }//id 3 camera
//                if(pair_index[3] >= 0)
//                {
//                    nInitialCorrespondences++;
//                    pFrame->mvbOutlierBKFs[i] = false;
//
//                    Eigen::Matrix<double,2,1> obs;
//
//                    const cv::KeyPoint &kpUn = pFrame->mvKeysMultipleUn[3][pair_index[3]];
//                    obs << kpUn.pt.x, kpUn.pt.y;
//
//                    g2o::EdgeSE3ProjectXYZOnlyPosePlus* e = new g2o::EdgeSE3ProjectXYZOnlyPosePlus();
//                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
//                    e->setMeasurement(obs);
//                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
//                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);
//
//                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//                    e->setRobustKernel(rk);
//                    rk->setDelta(deltaMono);
//
//                    e->fx = pFrame->vfx[3];
//                    e->fy = pFrame->vfy[3];
//                    e->cx = pFrame->vcx[3];
//                    e->cy = pFrame->vcy[3];
//                    cv::Mat Xw = pMP->GetWorldPos();
//                    e->Xw[0] = Xw.at<float>(0);
//                    e->Xw[1] = Xw.at<float>(1);
//                    e->Xw[2] = Xw.at<float>(2);
//                    cv::Mat tTcamji= cv::Mat::eye(4,4,CV_32F); //T30
//                    for(int x = 0; x < 3; x++)
//                    {
//                        tTcamji = pFrame->mvTcamji[x]* tTcamji;
//                    }
//                    e->Trl = Converter::toMatrix4d(tTcamji);
//                    optimizer.addEdge(e);
//                    vpEdgesMonoRight.push_back(e);
//                    vnIndexEdgeMonoRight.push_back(i);
//                }
            }
        }
    }

    cout<<"mappoint size in g2o: "<<mpcount<<endl;
    if(nInitialCorrespondences<3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    const int its[4]={10,10,10,10};
    int nBad=0;
    for(size_t it=0; it<4; it++)
    {

        vSE3->setEstimate(Converter::toSE3Quat(Frame.mTcw));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad=0;
        for(size_t i=0, iend=vpEdgesMono0.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono0[i];

            const size_t idx = vnIndexEdgeMono0[i];

            if(Frame.mvbOutlierBKFs[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                Frame.mvbOutlierBKFs[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                Frame.mvbOutlierBKFs[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesMono1.size(); i<iend; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPosePlus* e = vpEdgesMono1[i];

            const size_t idx = vnIndexEdgeMono1[i];

            if(Frame.mvbOutlierBKFs[idx])
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2>chi2Mono[it])
            {
                Frame.mvbOutlierBKFs[idx]=true;
                e->setLevel(1);
                nBad++;
            }
            else
            {
                Frame.mvbOutlierBKFs[idx]=false;
                e->setLevel(0);
            }

            if(it==2)
                e->setRobustKernel(0);
        }


        if(optimizer.edges().size()<10)
            break;
    }
    //cout<<"optimization edge size: "<< vpEdgesStereo.size() + vpEdgesMonoLeft.size() + vpEdgesMonoRight.size()<<" mono edge left:" <<vpEdgesMonoLeft.size()<<" stereo edge:" <<vpEdgesStereo.size()<<", right mono edge: "<<vpEdgesMonoRight.size()<<endl;
    // Recover optimized pose and return number of inliers
    g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    Frame.SetPose(pose);

    return nInitialCorrespondences-nBad;
}

void Optimizer::LocalBundleAdjustmentClient(kfptr pKF, bool* pbStopFlag, mapptr pMap, size_t ClientId, eSystemState SysState)
{
    // Local KeyFrames: Breath First Search from Current Keyframe
    list<kfptr> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mBALocalForKF = pKF->mId;

    const vector<kfptr> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        kfptr pKFi = vNeighKFs[i];
        pKFi->mBALocalForKF = pKF->mId;
        if(!pKFi->isBad())
        {
            lLocalKeyFrames.push_back(pKFi);
        }
    }

    // Local MapPoints seen in Local KeyFrames
    list<mpptr> lLocalMapPoints;
    for(list<kfptr>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<mpptr> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<mpptr>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            mpptr pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mBALocalForKF!=pKF->mId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mBALocalForKF=pKF->mId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<kfptr> lFixedCameras;
    for(list<mpptr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<kfptr,size_t> observations = (*lit)->GetObservations();
        for(map<kfptr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            kfptr pKFi = mit->first;

            if(pKFi->mBALocalForKF!=pKF->mId && pKFi->mBAFixedForKF!=pKF->mId)
            {
                pKFi->mBAFixedForKF=pKF->mId;
                if(!pKFi->isBad())
                {
                    lFixedCameras.push_back(pKFi);
                }
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    // Set Local KeyFrame vertices
    for(list<kfptr>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        kfptr pKFi = *lit;

        if(pKFi->mId.first >= IDRANGE)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
            throw infrastructure_ex();
        }

        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(Optimizer::GetID(pKFi->mId,true));
        vSE3->setFixed(pKFi->mId.first == 0 && pKFi->mId.second == ClientId);
        optimizer.addVertex(vSE3);
    }

    // 7. Set Fixed KeyFrame vertices
    for(list<kfptr>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        kfptr pKFi = *lit;

        if(pKFi->mId.first >= IDRANGE)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
            throw infrastructure_ex();
        }

        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(Optimizer::GetID(pKFi->mId,true));
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<kfptr> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<mpptr> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    //添加了双目的优化信息
    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<kfptr> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<mpptr> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    for(list<mpptr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        mpptr pMP = *lit;

        if(pMP->mId.first >= IDRANGE)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": MP index out of bounds" << endl;
            throw infrastructure_ex();
        }

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = Optimizer::GetID(pMP->mId,false);
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<kfptr,size_t> observations = pMP->GetObservations();

        //Set edges
        for(map<kfptr,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            kfptr pKFi = mit->first;

            if(pKFi->mId.first >= IDRANGE)
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
                throw infrastructure_ex();
            }

            if(!pKFi->isBad())
            {   
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                if(pKFi->mvuRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(Optimizer::GetID(pKFi->mId,true))));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else //添加双目观测
                {
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(Optimizer::GetID(pKFi->mId,true))));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);                    
                }
                
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {
        // 10 ：Check inlier observations
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            mpptr pMP = vpMapPointEdgeMono[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0); 
        }

        //在双目中检测outlier
    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        mpptr pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }        

        // Optimize again without the outliers

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
    }

    vector<pair<kfptr,mpptr> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        mpptr pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            kfptr pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }
    //重新计算误差，剔除优化不好的点
     for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        mpptr pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            kfptr pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi,pMP));
        }
    }   

    // Get Map Mutex
    if(SysState != eSystemState::SERVER)
        while(!pMap->LockMapUpdate()){usleep(params::timings::miLockSleep);}

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            kfptr pKFi = vToErase[i].first;
            mpptr pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for(list<kfptr>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        kfptr pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(Optimizer::GetID(pKF->mId,true)));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        
        //ADD
        Mat mTcwLBA_tmp = Converter::toCvMat(SE3quat)*pKF->GetPoseInverse();
        Eigen::Matrix4d testLBA = Converter::toMatrix4d(mTcwLBA_tmp);
        Eigen::Matrix4d poseBef = Converter::toMatrix4d(pKF->GetPoseInverse());
        Eigen::Matrix4d poseAfLBA = Converter::toMatrix4d(Converter::toCvMat(SE3quat).inv());

        if(testLBA.block<3,1>(0,3).norm()>0.5)
        {
            cout<<"第"<<pKF->mId.second<<"client, 第"<<pKF->mId.first<<"帧"<<" 时间戳:"<<setprecision(25)<<pKF->mTimeStamp<<"的LBA效果不好!!!!"<<endl;
            cout<<"LBA之前的位置： "<<poseBef.block<3,1>(0,3).transpose()<<endl;
            cout<<"LBA之后的位置： "<<poseAfLBA.block<3,1>(0,3).transpose()<<endl;
            cout<<"LBA效果不好 LBA中一共有"<<lLocalKeyFrames.size()<<"帧！"<<endl;
        }
        else
        {
            pKF->SetPose(Converter::toCvMat(SE3quat),false);
            
        }
        //END

        pKF->mbUpdatedByServer = false;
    }

    //Points
    for(list<mpptr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        mpptr pMP = *lit;
        if(pMP->isBad())
        {
            //maybe observations were erased by BA, and now MP has become bad
            mpptr pMPcheck = pMap->GetMpPtr(pMP->mId);
            if(pMPcheck)
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " MP bad but not erased from map" << endl;
                throw estd::infrastructure_ex();
            }
        }
        else
        {
            g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(Optimizer::GetID(pMP->mId,false)));
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()),false);
            pMP->UpdateNormalAndDepth();
        }
    }

    if(SysState != eSystemState::SERVER)
    {
        pMap->UnLockMapUpdate();
    }
}


void Optimizer::LocalBundleAdjustmentClient(bkfptr pBKFs, bool* pbStopFlag, bmapptr pBMap, size_t ClientId, eSystemState SysState)
{
    // Local BundledKeyFrames: Breath First Search from Current BundledKeyFrames
    list<bkfptr> lLocalBundledKeyFrames;

    lLocalBundledKeyFrames.push_back(pBKFs);
    pBKFs->mBALocalForBKFs = pBKFs->mId;

    const vector<bkfptr> vNeighBKFs = pBKFs->GetVectorCovisibleBundledKeyFrames();
    for(int i=0, iend=vNeighBKFs.size(); i<iend; i++)
    {
        bkfptr pBKFsi = vNeighBKFs[i];
        pBKFsi->mBALocalForBKFs = pBKFs->mId;
        if(!pBKFsi->isBad())
        {
            lLocalBundledKeyFrames.push_back(pBKFsi);
        }
    }

    // Local MapPoints seen in Local BundledKeyFrames
    list<mpptr> lLocalMapPoints;
    for(list<bkfptr>::iterator lit=lLocalBundledKeyFrames.begin() , lend=lLocalBundledKeyFrames.end(); lit!=lend; lit++)
    {
        vector<mpptr> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<mpptr>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            mpptr pMP = *vit;
            if(pMP)
                if(!pMP->isBad())
                    if(pMP->mBALocalForBKFs!=pBKFs->mId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mBALocalForBKFs=pBKFs->mId;
                    }
        }
    }

    // Fixed BundledKeyFrames. BundledKeyFrames that see Local MapPoints but that are not Local BundledKeyFrames
    list<bkfptr> lFixedCameras;
    for(list<mpptr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<bkfptr,size_t> observations = (*lit)->GetBKFsObservations();
        for(map<bkfptr,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            bkfptr pBKFsi = mit->first;

            if(pBKFsi->mBALocalForBKFs!=pBKFs->mId && pBKFsi->mBAFixedForBKFs!=pBKFs->mId)
            {
                pBKFsi->mBAFixedForBKFs=pBKFs->mId;
                if(!pBKFsi->isBad())
                {
                    lFixedCameras.push_back(pBKFsi);
                }
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    // Set Local BundledKeyFrames vertices

    for(list<bkfptr>::iterator lit=lLocalBundledKeyFrames.begin(), lend=lLocalBundledKeyFrames.end(); lit!=lend; lit++)
    {
        bkfptr pBKFsi = *lit;

        if(pBKFsi->mId.first >= IDRANGE)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": BKF index out of bounds" << endl;
            throw infrastructure_ex();
        }

        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pBKFsi->GetPose()));
        vSE3->setId(Optimizer::GetID(pBKFsi->mId,true));
        vSE3->setFixed(pBKFsi->mId.first == 0 && pBKFsi->mId.second == ClientId);
        optimizer.addVertex(vSE3);
    }

    // 7. Set Fixed BundledKeyFrames vertices
    for(list<bkfptr>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        bkfptr pBKFsi = *lit;

        if(pBKFsi->mId.first >= IDRANGE)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
            throw infrastructure_ex();
        }

        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pBKFsi->GetPose()));
        vSE3->setId(Optimizer::GetID(pBKFsi->mId,true));
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalBundledKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono0;
    vpEdgesMono0.reserve(nExpectedSize);

    vector<bkfptr> vpEdgeBKFsMono0;
    vpEdgeBKFsMono0.reserve(nExpectedSize);

    vector<mpptr> vpMapPointEdgeMono0; 
    vpMapPointEdgeMono0.reserve(nExpectedSize);

    //add multicamera 
    vector<g2o::EdgeSE3ProjectXYZPlus*> vpEdgesMono1;
    vpEdgesMono1.reserve(nExpectedSize);

    vector<bkfptr> vpEdgeBKFsMono1;
    vpEdgeBKFsMono1.reserve(nExpectedSize);

    vector<mpptr> vpMapPointEdgeMono1;
    vpMapPointEdgeMono1.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    for(list<mpptr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        mpptr pMP = *lit;

        if(pMP->mId.first >= IDRANGE)
        {
            cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": MP index out of bounds" << endl;
            throw infrastructure_ex();
        }

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = Optimizer::GetID(pMP->mId,false);
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<bkfptr,size_t> observations = pMP->GetBKFsObservations();

        //Set edges
        for(map<bkfptr,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            bkfptr pBKFsi = mit->first;

            if(pBKFsi->mId.first >= IDRANGE)
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m #1: In \"Optimizer::LocalBundleAdjustment(...)\": KF index out of bounds" << endl;
                throw infrastructure_ex();
            }

            if(!pBKFsi->isBad())
            {   

                vector<int> pair_index = pBKFsi->vKeyPointsIndexMapPlus[mit->second];

                for(int k = 0; k < pBKFs->cameraNum; k++)
                {
                    if(pair_index[k] >=0)
                    {
                        if(k == 0)  // mono camera 0
                        {
                            Eigen::Matrix<double,2,1> obs;
                            int idx = pair_index[k];
                            const cv::KeyPoint &kpUn = pBKFsi->mvKeysMultipleUn[0][idx];
                            obs << kpUn.pt.x, kpUn.pt.y;

                            g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(GetID(pBKFsi->mId,true))));
                            e->setMeasurement(obs);
                            const float &invSigma2 = pBKFsi->mvInvLevelSigma2[kpUn.octave];
                            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuberMono);

                            e->fx = pBKFsi->vfx[0];
                            e->fy = pBKFsi->vfy[0];
                            e->cx = pBKFsi->vcx[0];
                            e->cy = pBKFsi->vcy[0];

                            optimizer.addEdge(e);
                            vpEdgesMono0.push_back(e);
                            vpEdgeBKFsMono0.push_back(pBKFsi);
                            vpMapPointEdgeMono0.push_back(pMP);
                        }
                        else  //mono camera 1~3
                        {
                            Eigen::Matrix<double,2,1> obs;
                            int idx = pair_index[k];
                            const cv::KeyPoint &kpUn = pBKFsi->mvKeysMultipleUn[k][idx];
                            obs << kpUn.pt.x, kpUn.pt.y;

                            g2o::EdgeSE3ProjectXYZPlus* e = new g2o::EdgeSE3ProjectXYZPlus();

                            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(GetID(pBKFsi->mId,true))));
                            e->setMeasurement(obs);
                            const float &invSigma2 = pBKFsi->mvInvLevelSigma2[kpUn.octave];
                            e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                            e->setRobustKernel(rk);
                            rk->setDelta(thHuberMono);

                            e->fx = pBKFsi->vfx[k];
                            e->fy = pBKFsi->vfy[k];
                            e->cx = pBKFsi->vcx[k];
                            e->cy = pBKFsi->vcy[k];
                            e->Trl = Converter::toMatrix4d(pBKFs->vmTi0[k]);
                            optimizer.addEdge(e);
                            vpEdgesMono1.push_back(e);
                            vpEdgeBKFsMono1.push_back(pBKFsi);
                            vpMapPointEdgeMono1.push_back(pMP);
                        }   
                    } 
                }  
            }
        }
    }

    if(pbStopFlag)
        if(*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
        if(*pbStopFlag)
            bDoMore = false;

    if(bDoMore)
    {
        // 10 ：Check inlier observations
        for(size_t i=0, iend=vpEdgesMono0.size(); i<iend;i++)
        {
            g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono0[i];
            mpptr pMP = vpMapPointEdgeMono0[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);// 不优化
            }

            e->setRobustKernel(0); // 不使用核函数
        }

        //在双目中检测outlier
    for(size_t i=0, iend=vpEdgesMono1.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZPlus* e = vpEdgesMono1[i];
        mpptr pMP = vpMapPointEdgeMono1[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            e->setLevel(1);
        }

        e->setRobustKernel(0);
    }        

        // Optimize again without the outliers

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);
    }

    vector<pair<bkfptr,mpptr> > vToErase;
    vToErase.reserve(vpEdgesMono0.size()+vpEdgesMono1.size());

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono0.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono0[i];
        mpptr pMP = vpMapPointEdgeMono0[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            bkfptr pBKFsi = vpEdgeBKFsMono0[i];
            vToErase.push_back(make_pair(pBKFsi,pMP));
        }
    }
    //重新计算误差，剔除优化不好的点
     for(size_t i=0, iend=vpEdgesMono1.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZPlus* e = vpEdgesMono1[i];
        mpptr pMP = vpMapPointEdgeMono1[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            bkfptr pBKFsi = vpEdgeBKFsMono1[i];
            vToErase.push_back(make_pair(pBKFsi,pMP));
        }
    }   

    // Get Map Mutex
    if(SysState != eSystemState::SERVER)
        while(!pBMap->LockBMapUpdate()){usleep(params::timings::miLockSleep);}

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            bkfptr pBKFsi = vToErase[i].first;
            mpptr pMPi = vToErase[i].second;
            pBKFsi->EraseMapPointMatch(pMPi);
            pMPi->EraseBKFsObservation(pBKFsi,pBKFsi->cameraNum);
        }
    }

    // Recover optimized data

    //BundledKeyframes
    for(list<bkfptr>::iterator lit=lLocalBundledKeyFrames.begin(), lend=lLocalBundledKeyFrames.end(); lit!=lend; lit++)
    {
        bkfptr pBKFs = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(Optimizer::GetID(pBKFs->mId,true)));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        
        //ADD
        Mat mTcwLBA_tmp = Converter::toCvMat(SE3quat)*pBKFs->GetPoseInverse();
        Eigen::Matrix4d testLBA = Converter::toMatrix4d(mTcwLBA_tmp);
        Eigen::Matrix4d poseBef = Converter::toMatrix4d(pBKFs->GetPoseInverse());
        Eigen::Matrix4d poseAfLBA = Converter::toMatrix4d(Converter::toCvMat(SE3quat).inv());

        if(testLBA.block<3,1>(0,3).norm()>0.5)
        {
            cout<<"第"<<pBKFs->mId.second<<"client, 第"<<pBKFs->mId.first<<"帧"<<" 时间戳:"<<setprecision(25)<<pBKFs->mTimeStamp<<"的LBA效果不好!!!!"<<endl;
            cout<<"LBA之前的位置： "<<poseBef.block<3,1>(0,3).transpose()<<endl;
            cout<<"LBA之后的位置： "<<poseAfLBA.block<3,1>(0,3).transpose()<<endl;
            cout<<"LBA效果不好 LBA中一共有"<<lLocalBundledKeyFrames.size()<<"帧！"<<endl;
        }
        else
        {
            pBKFs->SetPose(Converter::toCvMat(SE3quat),false);
            
        }
        //END

        pBKFs->mbUpdatedByServer = false;
    }

    //Points
    for(list<mpptr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        mpptr pMP = *lit;
        if(pMP->isBad())
        {
            //maybe observations were erased by BA, and now MP has become bad
            mpptr pMPcheck = pBMap->GetMpPtr(pMP->mId);
            if(pMPcheck)
            {
                cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << ":" << __LINE__ << " MP bad but not erased from map" << endl;
                throw estd::infrastructure_ex();
            }
        }
        else
        {
            g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(Optimizer::GetID(pMP->mId,false)));
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()),false);
            pMP->UpdateNormalAndDepthPlus();
        }
    }

    if(SysState != eSystemState::SERVER)
    {
        pBMap->UnLockBMapUpdate();
    }
}
//以上的函数都修改加入了双目优化信息
//加入了双目在server
void Optimizer::MapFusionGBA(mapptr pMap, size_t ClientId, int nIterations, bool* pbStopFlag, idpair nLoopKF, const bool bRobust)
{
    stringstream* ss;
    ss = new stringstream;
    for(set<size_t>::iterator sit = pMap->msuAssClients.begin();sit != pMap->msuAssClients.end();++sit)
        *ss << *sit << ";";
    cout << "--> Optimizing Map " << pMap->mMapId << " -- Contains Agents " << ss->str() << endl;
    delete ss;

    //prepare structures

    vector<kfptr> vpKFs = pMap->GetAllKeyFrames();
    vector<mpptr> vpMP = pMap->GetAllMapPoints();

    const idpair zeropair = make_pair(0,pMap->mMapId);

    if(pMap->mvpKeyFrameOrigins.empty())
    {
        cout << "\033[1;31m!!!!! ERROR !!!!!\033[0m " << __func__ << __LINE__ << " pMap->mvpKeyFrameOrigins.empty()" << endl;
        throw infrastructure_ex();
    }

    idpair FixedId = (*(pMap->mvpKeyFrameOrigins.begin()))->mId;

    //GBA

    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size(),false);

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    size_t maxKFid = 0;

    // Set KeyFrame vertices
    cout << "----- Add KFs" << endl;
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        kfptr pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mUniqueId);

        vSE3->setFixed(pKF->mId==FixedId);
        optimizer.addVertex(vSE3);
        if(pKF->mUniqueId>maxKFid)
            maxKFid=pKF->mUniqueId;
    }

    const float thHuber2D = sqrt(5.99);
    
    //ADD
    const float thHuber3D = sqrt(7.815);
    //END

    // Set MapPoint vertices
    cout << "----- Add MPs" << endl;
    for(size_t i=0; i<vpMP.size(); i++)
    {
        mpptr pMP = vpMP[i];
        if(pMP->isBad())
            continue;

        const map<kfptr,size_t> observations = pMP->GetObservations();
        if (observations.size() < 2) {
          vbNotIncludedMP[i] = true;
          continue;
        }
        int nEdges = 0;
        const size_t id = pMP->mUniqueId;

        // Do a pre-check to ensure at least 2 proper observations
        for(map<kfptr,size_t>::const_iterator mit = observations.begin(); mit != observations.end(); ++mit) {
          kfptr pKF = mit->first;
          if(!pKF || pKF->isBad() || pKF->mUniqueId>maxKFid) continue;
          nEdges++;
        }

        if (nEdges < 2) {
          vbNotIncludedMP[i] = true;
          continue;
        }

        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));

        vPoint->setId(id);

        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        //SET EDGES
        for(map<kfptr,size_t>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {

            kfptr pKF = mit->first;
            if(!pKF || pKF->isBad() || pKF->mUniqueId>maxKFid)
                continue;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

            //ADD mono
            if(pKF->mvuRight[mit->second]<0)
            {
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mUniqueId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            }
            else // stereo
            {
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mUniqueId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                if(bRobust)
                {
                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);
            }
            //END
        }
    }

    // Optimize!
    cout << "----- Optimize" << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    cout << "----- Recover KFs" << endl;
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        kfptr pKF = vpKFs[i];
        if(pKF->isBad())
            continue;

        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mUniqueId));
        g2o::SE3Quat SE3quat = vSE3->estimate();

        if(nLoopKF==zeropair)
        {
            pKF->SetPose(Converter::toCvMat(SE3quat),true);
        }
        else
        {
            pKF->mTcwGBA.create(4,4,CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            //Add
            Mat mTcwGBA_tmp = Converter::toCvMat(SE3quat)*pKF->GetPoseInverse();
            Eigen::Matrix4d testGBA = Converter::toMatrix4d(mTcwGBA_tmp);
            Eigen::Matrix4d poseBef = Converter::toMatrix4d(pKF->GetPoseInverse());
            Eigen::Matrix4d poseAfGBA = Converter::toMatrix4d(Converter::toCvMat(SE3quat).inv());

            if(testGBA.block<3,1>(0,3).norm()>0.5)
            {
                cout<<"第"<<pKF->mId.second<<" client, 第"<<pKF->mId.first<<" client"<<"时间戳:"<<setprecision(25)<<pKF->mTimeStamp<<"的GPA效果不好!!!!"<<endl;
                cout<<"GBA之前的位置： "<<poseBef.block<3,1>(0,3).transpose()<<endl;
                cout<<"GBA之后的位置： "<<poseAfGBA.block<3,1>(0,3).transpose()<<endl;
            }
            //End
            pKF->mBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    cout << "----- Recover MPs" << endl;
    for(size_t i=0; i<vpMP.size(); i++)
    {
        if(vbNotIncludedMP[i])
            continue;

        mpptr pMP = vpMP[i];

        if(pMP->isBad())
            continue;

        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mUniqueId));

        if(nLoopKF==zeropair)
        {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()),true);
            pMP->UpdateNormalAndDepth();
        }
        else
        {
            pMP->mPosGBA.create(3,1,CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mBAGlobalForKF = nLoopKF;
        }
    }
}

int Optimizer::OptimizeSim3(kfptr pKF1, kfptr pKF2, std::vector<mpptr> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, bool bFixScale)
{
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Calibration
    const cv::Mat &K1 = pKF1->mK;
    const cv::Mat &K2 = pKF2->mK;

    // Camera poses
    const cv::Mat R1w = pKF1->GetRotation();
    const cv::Mat t1w = pKF1->GetTranslation();
    const cv::Mat R2w = pKF2->GetRotation();
    const cv::Mat t2w = pKF2->GetTranslation();

    // Set Sim3 vertex
    g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();
    vSim3->_fix_scale=bFixScale;
    vSim3->setEstimate(g2oS12);
    vSim3->setId(0);
    vSim3->setFixed(false);
    vSim3->_principle_point1[0] = K1.at<float>(0,2);
    vSim3->_principle_point1[1] = K1.at<float>(1,2);
    vSim3->_focal_length1[0] = K1.at<float>(0,0);
    vSim3->_focal_length1[1] = K1.at<float>(1,1);
    vSim3->_principle_point2[0] = K2.at<float>(0,2);
    vSim3->_principle_point2[1] = K2.at<float>(1,2);
    vSim3->_focal_length2[0] = K2.at<float>(0,0);
    vSim3->_focal_length2[1] = K2.at<float>(1,1);
    optimizer.addVertex(vSim3);

    // Set MapPoint vertices
    const int N = vpMatches1.size();
    const vector<mpptr> vpMapPoints1 = pKF1->GetMapPointMatches();
    vector<g2o::EdgeSim3ProjectXYZ*> vpEdges12;
    vector<g2o::EdgeInverseSim3ProjectXYZ*> vpEdges21;
    vector<size_t> vnIndexEdge;

    vnIndexEdge.reserve(2*N);
    vpEdges12.reserve(2*N);
    vpEdges21.reserve(2*N);

    const float deltaHuber = sqrt(th2);

    int nCorrespondences = 0;

    for(int i=0; i<N; i++)
    {
        if(!vpMatches1[i])
            continue;

        mpptr pMP1 = vpMapPoints1[i];
        mpptr pMP2 = vpMatches1[i];

        const int id1 = 2*i+1;
        const int id2 = 2*(i+1);

        const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

        if(pMP1 && pMP2)
        {
            if(!pMP1->isBad() && !pMP2->isBad() && i2>=0)
            {
                g2o::VertexSBAPointXYZ* vPoint1 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D1w = pMP1->GetWorldPos();
                cv::Mat P3D1c = R1w*P3D1w + t1w;
                vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                vPoint1->setId(id1);
                vPoint1->setFixed(true);
                optimizer.addVertex(vPoint1);

                g2o::VertexSBAPointXYZ* vPoint2 = new g2o::VertexSBAPointXYZ();
                cv::Mat P3D2w = pMP2->GetWorldPos();
                cv::Mat P3D2c = R2w*P3D2w + t2w;
                vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                vPoint2->setId(id2);
                vPoint2->setFixed(true);
                optimizer.addVertex(vPoint2);
            }
            else
                continue;
        }
        else
            continue;

        nCorrespondences++;

        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
        obs1 << kpUn1.pt.x, kpUn1.pt.y;

        g2o::EdgeSim3ProjectXYZ* e12 = new g2o::EdgeSim3ProjectXYZ();
        e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id2)));
        e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e12->setMeasurement(obs1);
        const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
        e12->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare1);

        g2o::RobustKernelHuber* rk1 = new g2o::RobustKernelHuber;
        e12->setRobustKernel(rk1);
        rk1->setDelta(deltaHuber);
        optimizer.addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
        obs2 << kpUn2.pt.x, kpUn2.pt.y;

        g2o::EdgeInverseSim3ProjectXYZ* e21 = new g2o::EdgeInverseSim3ProjectXYZ();

        e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id1)));
        e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        e21->setMeasurement(obs2);
        float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
        e21->setInformation(Eigen::Matrix2d::Identity()*invSigmaSquare2);

        g2o::RobustKernelHuber* rk2 = new g2o::RobustKernelHuber;
        e21->setRobustKernel(rk2);
        rk2->setDelta(deltaHuber);
        optimizer.addEdge(e21);

        vpEdges12.push_back(e12);
        vpEdges21.push_back(e21);
        vnIndexEdge.push_back(i);
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // Check inliers
    int nBad=0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2*0.8 || e21->chi2()>th2*0.8)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<mpptr>(NULL);
            optimizer.removeEdge(e12);
            optimizer.removeEdge(e21);
            vpEdges12[i]=static_cast<g2o::EdgeSim3ProjectXYZ*>(NULL);
            vpEdges21[i]=static_cast<g2o::EdgeInverseSim3ProjectXYZ*>(NULL);
            nBad++;
        }
    }

    int nMoreIterations;
    if(nBad>0)
        nMoreIterations=10;
    else
        nMoreIterations=5;

    if(nCorrespondences-nBad<10)
        return 0;

    // Optimize again only with inliers

    optimizer.initializeOptimization();
    optimizer.optimize(nMoreIterations);

    int nIn = 0;
    for(size_t i=0; i<vpEdges12.size();i++)
    {
        g2o::EdgeSim3ProjectXYZ* e12 = vpEdges12[i];
        g2o::EdgeInverseSim3ProjectXYZ* e21 = vpEdges21[i];
        if(!e12 || !e21)
            continue;

        if(e12->chi2()>th2*0.8 || e21->chi2()>th2*0.8)
        {
            size_t idx = vnIndexEdge[i];
            vpMatches1[idx]=static_cast<mpptr>(NULL);
        }
        else
            nIn++;
    }

    // Recover optimized Sim3
    g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(0));
    g2oS12= vSim3_recov->estimate();

    return nIn;
}

void Optimizer::OptimizeEssentialGraphLoopClosure(mapptr pMap, kfptr pLoopKF, kfptr pCurKF,
                                       const KeyFrameAndPose &NonCorrectedSim3,
                                       const KeyFrameAndPose &CorrectedSim3,
                                       const map<kfptr, set<kfptr> > &LoopConnections, const bool &bFixScale)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<kfptr> vpKFs = pMap->GetAllKeyFrames();
    const vector<mpptr> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFidUnique(); //vpKFs.size();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    const int minFeat = params::opt::miEssGraphMinFeats;

    // Set KeyFrame vertices
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        kfptr pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const size_t nIDi = pKF->mUniqueId;

        KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

        if(it!=CorrectedSim3.end())
        {
            vScw[nIDi] = it->second;
            VSim3->setEstimate(it->second);
        }
        else
        {
            Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
            Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
            g2o::Sim3 Siw(Rcw,tcw,1.0);
            vScw[nIDi] = Siw;
            VSim3->setEstimate(Siw);
        }

        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }

    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // Set Loop edges
    for(map<kfptr , set<kfptr > >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        kfptr pKF = mit->first;

        if(pKF->isBad()) continue;

        const size_t nIDi = pKF->mUniqueId;

        const set<kfptr> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for(set<kfptr>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            if((*sit)->isBad()) continue;
            const size_t nIDj = (*sit)->mUniqueId;

            if((nIDi!=pCurKF->mUniqueId || nIDj!=pLoopKF->mUniqueId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        kfptr pKF = vpKFs[i];

        const size_t nIDi = pKF->mUniqueId;

        g2o::Sim3 Swi;

        KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

        if(iti!=NonCorrectedSim3.end())
            Swi = (iti->second).inverse();
        else
            Swi = vScw[nIDi].inverse();

        kfptr pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            const size_t nIDj = pParentKF->mUniqueId;

            g2o::Sim3 Sjw;

            KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

            if(itj!=NonCorrectedSim3.end())
                Sjw = itj->second;
            else
                Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const set<kfptr> sLoopEdges = pKF->GetLoopEdges();
        for(set<kfptr>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            kfptr pLKF = *sit;

            const size_t nIDj = pLKF->mUniqueId;

            if(nIDj<nIDi)
            {
                g2o::Sim3 Slw;

                KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                if(itl!=NonCorrectedSim3.end())
                    Slw = itl->second;
                else
                    Slw = vScw[nIDj];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const vector<kfptr> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<kfptr>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            kfptr pKFn = *vit;

            if((*vit)->isBad()) continue;

            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                const size_t nIDj = pKFn->mUniqueId;

                if(!pKFn->isBad() && nIDj<nIDi)
                {
                    if(sInsertedEdges.count(make_pair(min(nIDi,nIDj),max(nIDi,nIDj))))
                        continue;

                    g2o::Sim3 Snw;

                    KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                    if(itn!=NonCorrectedSim3.end())
                        Snw = itn->second;
                    else
                        Snw = vScw[nIDj];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        kfptr pKFi = vpKFs[i];

        const size_t nIDi = pKFi->mUniqueId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(pKFi->mUniqueId));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(Tiw,true);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        mpptr pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        size_t nIDr;
        if(pMP->mCorrectedByKF_LC==pCurKF->mId)
        {
            nIDr = pMP->mCorrectedReference_LC;
        }
        else
        {
            kfptr pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mUniqueId;
        }

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw,true);

        pMP->UpdateNormalAndDepth();
    }
}

void Optimizer::OptimizeEssentialGraphMapFusion(mapptr pMap, kfptr pLoopKF, kfptr pCurKF,
                                                  const map<kfptr, set<kfptr> > &LoopConnections, const bool &bFixScale)
{
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    g2o::BlockSolver_7_3::LinearSolverType * linearSolver =
           new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    const vector<kfptr> vpKFs = pMap->GetAllKeyFrames();
    const vector<mpptr> vpMPs = pMap->GetAllMapPoints();

    const unsigned int nMaxKFid = pMap->GetMaxKFidUnique();

    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vScw(nMaxKFid+1);
    vector<g2o::Sim3,Eigen::aligned_allocator<g2o::Sim3> > vCorrectedSwc(nMaxKFid+1);
    vector<g2o::VertexSim3Expmap*> vpVertices(nMaxKFid+1);

    const int minFeat = params::opt::miEssGraphMinFeats;

    // Set KeyFrame vertices
    for(size_t i=0, iend=vpKFs.size(); i<iend;i++)
    {
        kfptr pKF = vpKFs[i];
        if(pKF->isBad())
            continue;
        g2o::VertexSim3Expmap* VSim3 = new g2o::VertexSim3Expmap();

        const size_t nIDi = pKF->mUniqueId;

        Eigen::Matrix<double,3,3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
        Eigen::Matrix<double,3,1> tcw = Converter::toVector3d(pKF->GetTranslation());
        g2o::Sim3 Siw(Rcw,tcw,1.0);
        vScw[nIDi] = Siw;
        VSim3->setEstimate(Siw);

        if(pKF==pLoopKF)
            VSim3->setFixed(true);

        VSim3->setId(nIDi);
        VSim3->setMarginalized(false);
        VSim3->_fix_scale = bFixScale;

        optimizer.addVertex(VSim3);

        vpVertices[nIDi]=VSim3;
    }

    set<pair<long unsigned int,long unsigned int> > sInsertedEdges;

    const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

    // Set Loop edges
    for(map<kfptr , set<kfptr > >::const_iterator mit = LoopConnections.begin(), mend=LoopConnections.end(); mit!=mend; mit++)
    {
        kfptr pKF = mit->first;

        if(pKF->isBad()) continue;

        const size_t nIDi = pKF->mUniqueId;

        const set<kfptr> &spConnections = mit->second;
        const g2o::Sim3 Siw = vScw[nIDi];
        const g2o::Sim3 Swi = Siw.inverse();

        for(set<kfptr>::const_iterator sit=spConnections.begin(), send=spConnections.end(); sit!=send; sit++)
        {
            if((*sit)->isBad()) continue;

            const size_t nIDj = (*sit)->mUniqueId;

            if((nIDi!=pCurKF->mUniqueId || nIDj!=pLoopKF->mUniqueId) && pKF->GetWeight(*sit)<minFeat)
                continue;

            const g2o::Sim3 Sjw = vScw[nIDj];
            const g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;

            optimizer.addEdge(e);

            sInsertedEdges.insert(make_pair(min(nIDi,nIDj),max(nIDi,nIDj)));
        }
    }

    // Set normal edges
    for(size_t i=0, iend=vpKFs.size(); i<iend; i++)
    {
        kfptr pKF = vpKFs[i];

        const size_t nIDi = pKF->mUniqueId;

        g2o::Sim3 Swi = vScw[nIDi].inverse();

        kfptr pParentKF = pKF->GetParent();

        // Spanning tree edge
        if(pParentKF)
        {
            const size_t nIDj = pParentKF->mUniqueId;

            g2o::Sim3 Sjw = vScw[nIDj];

            g2o::Sim3 Sji = Sjw * Swi;

            g2o::EdgeSim3* e = new g2o::EdgeSim3();
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
            e->setMeasurement(Sji);

            e->information() = matLambda;
            optimizer.addEdge(e);
        }

        // Loop edges
        const set<kfptr> sLoopEdges = pKF->GetLoopEdges();
        for(set<kfptr>::const_iterator sit=sLoopEdges.begin(), send=sLoopEdges.end(); sit!=send; sit++)
        {
            kfptr pLKF = *sit;

            const size_t nIDj = pLKF->mUniqueId;

            if(nIDj<nIDi)
            {
                g2o::Sim3 Slw = vScw[nIDj];

                g2o::Sim3 Sli = Slw * Swi;
                g2o::EdgeSim3* el = new g2o::EdgeSim3();
                el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
                el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                el->setMeasurement(Sli);
                el->information() = matLambda;
                optimizer.addEdge(el);
            }
        }

        // Covisibility graph edges
        const vector<kfptr> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
        for(vector<kfptr>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++)
        {
            kfptr pKFn = *vit;

            if((*vit)->isBad()) continue;

            if(pKFn && pKFn!=pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
            {
                const size_t nIDj = pKFn->mUniqueId;

                if(!pKFn->isBad() && nIDj<nIDi)
                {
                    if(sInsertedEdges.count(make_pair(min(nIDi,nIDj),max(nIDi,nIDj))))
                        continue;

                    g2o::Sim3 Snw = vScw[nIDj];

                    g2o::Sim3 Sni = Snw * Swi;

                    g2o::EdgeSim3* en = new g2o::EdgeSim3();
                    en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDj)));
                    en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nIDi)));
                    en->setMeasurement(Sni);
                    en->information() = matLambda;
                    optimizer.addEdge(en);
                }
            }
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
    for(size_t i=0;i<vpKFs.size();i++)
    {
        kfptr pKFi = vpKFs[i];
        const size_t nIDi = pKFi->mUniqueId;

        g2o::VertexSim3Expmap* VSim3 = static_cast<g2o::VertexSim3Expmap*>(optimizer.vertex(pKFi->mUniqueId));
        g2o::Sim3 CorrectedSiw =  VSim3->estimate();
        vCorrectedSwc[nIDi]=CorrectedSiw.inverse();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        double s = CorrectedSiw.scale();

        eigt *=(1./s); //[R t/s;0 1]

        cv::Mat Tiw = Converter::toCvSE3(eigR,eigt);

        pKFi->SetPose(Tiw,true);
    }

    // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
    for(size_t i=0, iend=vpMPs.size(); i<iend; i++)
    {
        mpptr pMP = vpMPs[i];

        if(pMP->isBad())
            continue;

        int nIDr;
        if(pMP->mCorrectedByKF_MM==pCurKF->mId)
        {
            nIDr = pMP->mCorrectedReference_MM;
        }
        else
        {
            kfptr pRefKF = pMP->GetReferenceKeyFrame();
            nIDr = pRefKF->mUniqueId;
        }

        g2o::Sim3 Srw = vScw[nIDr];
        g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

        cv::Mat P3Dw = pMP->GetWorldPos();
        Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        Eigen::Matrix<double,3,1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

        cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        pMP->SetWorldPos(cvCorrectedP3Dw,true);

        pMP->UpdateNormalAndDepth();
    }
}

} // end ns
