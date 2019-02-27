#include "optimizer.h"
#include "converter.h"
#include <iostream>

using namespace std;

void Optimizer::BundleAdjustment(cv::Mat& Tcw1, cv::Mat& Tcw2, std::vector<cv::Point3f>& pts3d, std::vector<cv::Point2f> pts1, std::vector<cv::Point2f> pts2, int nIterations)
{

    //步骤1：初始化g2o优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    //linear solver
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    //linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();

    //algorithm, LM
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );
    optimizer.setAlgorithm(algorithm);


    //步骤2：准备相机参数
    float fx = 712.58465;
    float fy = 713.57849;
    float cx = 613.71289;
    float cy = 386.50469;

    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );


    //步骤3：向优化器添加顶点 set vertices: 
    //camera_pose ---> g2o::VertexSE3Expmap 
    //map_points ---> g2o::VertexSBAPointXYZ

    g2o::VertexSE3Expmap* v1 = new g2o::VertexSE3Expmap();
    v1->setId(0);        
    v1->setFixed(true);   
    v1->setEstimate(Converter::toSE3Quat(Tcw1));
    optimizer.addVertex(v1);
    
    g2o::VertexSE3Expmap* v2 = new g2o::VertexSE3Expmap();
    v2->setId(1);        
    v2->setEstimate(Converter::toSE3Quat(Tcw2));
    optimizer.addVertex(v2);


    for(int i=0; i<pts3d.size(); i++)
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId( 2 + i );
        v->setMarginalized(true);
        v->setEstimate(Eigen::Vector3d(pts3d[i].x,pts3d[i].y,pts3d[i].z));
        optimizer.addVertex( v );        
    }

    //步骤4：向优化器添加投影边边 
    //projection relationship ---> g2o::EdgeProjectXYZ2UV
    vector<g2o::EdgeProjectXYZ2UV*> edges;
    for ( size_t i=0; i<pts1.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(0)) );
        edge->setMeasurement( Eigen::Vector2d(pts1[i].x, pts1[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0, 0);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }

    // 第二帧
    for ( size_t i=0; i<pts2.size(); i++ )
    {
        g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(i+2)) );
        edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(1)) );
        edge->setMeasurement( Eigen::Vector2d(pts2[i].x, pts2[i].y ) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        edge->setParameterId(0,0);
        // 核函数
        edge->setRobustKernel( new g2o::RobustKernelHuber() );
        optimizer.addEdge( edge );
        edges.push_back(edge);
    }
    
    cout<<"开始优化"<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);
    cout<<"优化完毕"<<endl;


    //我们比较关心两帧之间的变换矩阵
    g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(1) );
    g2o::SE3Quat pose = v->estimate();
    Tcw2 = Converter::toCvMat(pose);
    //cout<<"Pose="<<endl<<Tcw2<<endl;

    // 以及所有特征点的位置
    for ( size_t i=0; i<pts3d.size(); i++ )
    {
        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2));
        //cout<<"vertex id "<<i+2<<", pos = ";
        Eigen::Vector3d pos = v->estimate();
        //cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
        pts3d[i] = cv::Point3f(pos(0), pos(1), pos(2));
    }
}


void Optimizer::BundleAdjustment(std::vector<Frame*> Frames, std::vector<Seed*> Seeds, int nIterations)
{

    //步骤1：初始化g2o优化器
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);

    //linear solver
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    //linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();

    //algorithm, LM
    g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );
    optimizer.setAlgorithm(algorithm);


    //步骤2：准备相机参数
    float fx = Frames[0]->fx;;
    float cx = Frames[0]->cx;
    float cy = Frames[0]->cy;

    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );


    // //步骤3：向优化器添加顶点 set vertices: 
    int n_frames = Frames.size();

    for(int i=0; i<n_frames; i++)
    {
        g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
        v->setId(i);        
        if(i==0)
            v->setFixed(true);   
        v->setEstimate(Converter::toSE3Quat(Frames[i]->GetPose()));
        optimizer.addVertex(v);
    }

    int n_seeds = Seeds.size();

    for(int i=0; i<n_seeds; i++)
    {
        g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
        v->setId( n_frames + i );
        v->setMarginalized(true);
        v->setEstimate(Converter::toVector3d(Seeds[i]->GetWorldPose()));
        optimizer.addVertex( v );        
    }


    //步骤4：向优化器添加投影边边 
    for(int i=0; i<n_frames; i++)
    {
        for(int j=0; j<n_seeds; j++)
        {
            g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
            edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(j+n_frames)) );
            edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(i)) );

            for(int k=0; k<Frames[i]->GetTrafficSigns()->mVertices.size(); k++)         //遍歷所有畫面中的角點   
            {
                if(Seeds[j]->mType == Frames[i]->GetTrafficSigns()->mTypes[k])
                {
                    cv::Point2f pt = Frames[i]->GetTrafficSigns()->mVertices[k];
                    edge->setMeasurement( Eigen::Vector2d(pt.x , pt.y) );
                    edge->setInformation( Eigen::Matrix2d::Identity() );
                    edge->setParameterId(0,0);
                    // 核函数
                    edge->setRobustKernel( new g2o::RobustKernelHuber() );
                    optimizer.addEdge( edge );
                }
            }
        }
    }

    //步骤5：开始优化
    cout<<"开始优化"<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);
    cout<<"优化完毕"<<endl;


    for(int i=0; i<n_frames; i++) //update pose
    {
        g2o::VertexSE3Expmap* v = dynamic_cast<g2o::VertexSE3Expmap*>( optimizer.vertex(i) );
        g2o::SE3Quat pose = v->estimate();
        Frames[i]->SetPose(Converter::toCvMat(pose));
    }   

    for ( int i=0; i<n_seeds; i++ )
    {
        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+n_frames));
        Eigen::Vector3d pos = v->estimate();
        cv::Point3f Pw = cv::Point3f(pos(0), pos(1), pos(2));
        cv::Mat Pc_ = Seeds[i]->mframe->GetRotation()*Converter::toCvMat(Pw) + Seeds[i]->mframe->GetTranslation();
        cv::Point3f Pc = Converter::toCvPoint3f(Pc_);
        Seeds[i]->mu = 1.0/norm(Pc);
    }
}