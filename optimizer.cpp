#include "optimizer.h"
#include "converter.h"
#include <iostream>

using namespace std;

Optimizer::Optimizer()
{
	cout<<"Construcr Optimizer"<<endl;
}

void Optimizer::BundleAdjustment(const std::vector<cv::Mat>& T_c_ws, const std::vector<cv::Point3f>& MapPoints, 
						const std::vector<std::vector<cv::Point2d>> vertexss, const solverType& sType, const cv::Mat& K)
{


























	cout<<"Bundle Adjustment"<<endl;

	//步骤1：初始化g2o优化器
	g2o::SparseOptimizer optimizer;
	optimizer.setVerbose(false);


	//linear solver
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
	if(sType==DENSE)
		linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
	else if (sType==CHOLMOD)
		linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();


	//algorithm, L-M method
	g2o::OptimizationAlgorithmLevenberg* algorithm = new g2o::OptimizationAlgorithmLevenberg(
		g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
	);
	optimizer.setAlgorithm(algorithm);


	//步骤2：准备相机参数
	double fx = K.at<double>(0,0);
	double cx = K.at<double>(0,2);
	double cy = K.at<double>(1,2);

    g2o::CameraParameters* camera = new g2o::CameraParameters( fx, Eigen::Vector2d(cx, cy), 0 );
    camera->setId(0);
    optimizer.addParameter( camera );



 	//步骤3：向优化器添加顶点 set vertices
 	//camera pose ---> g2o::VertexSE3Expmap
    int n_poses = T_c_ws.size();

    for(int i=0; i<n_poses; i++)
    {
    	g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
		v->setId(i);
		if(i==0)
			v->setFixed(true);    
		v->setEstimate(Converter::toSE3Quat(T_c_ws[i]));
		optimizer.addVertex(v);
    }


    //MapPoints ---> g2o::VertexSBAPointXYZ
    int n_points = MapPoints.size();

    for(int i=0; i<n_points; i++)
    {
    	g2o::VertexSBAPointXYZ* v = new g2o::VertexSBAPointXYZ();
    	v->setId( n_poses + i );
        v->setMarginalized(true);
        v->setEstimate(Converter::toVector3d(MapPoints[i]));
        optimizer.addVertex( v );
	}


    //步骤4：向优化器添加投影边边   
    //projection relationship ---> g2o::EdgeProjectXYZ2UV
	vector<g2o::EdgeProjectXYZ2UV*> edges;
	for(int i=0; i<n_poses; i++)
	{
		for(int j=0; j<n_points; j++)
    	{
    		g2o::EdgeProjectXYZ2UV*  edge = new g2o::EdgeProjectXYZ2UV();
        	edge->setVertex( 0, dynamic_cast<g2o::VertexSBAPointXYZ*>   (optimizer.vertex(j+n_poses)) );
        	edge->setVertex( 1, dynamic_cast<g2o::VertexSE3Expmap*>     (optimizer.vertex(i)) );
	        edge->setMeasurement( Eigen::Vector2d(vertexss[i][j].x , vertexss[i][j].y) );
	        edge->setInformation( Eigen::Matrix2d::Identity() );
	        edge->setParameterId(0,0);
	        edge->setRobustKernel( new g2o::RobustKernelHuber() );
	        optimizer.addEdge( edge );
	        edges.push_back(edge);
		}
	}


    //所有特征点的位置
    for (int i=0; i<n_points; i++)
    {
        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+n_poses));
        cout<<"vertex id "<<i+n_poses<<", pos = ";
        Eigen::Vector3d pos = v->estimate();
        cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
    }

    cout<<"开始优化"<<endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();


    bool STRUCTURE_ONLY = true;
	if (STRUCTURE_ONLY)
	{
	   	g2o::StructureOnlySolver<3> structure_only_ba;
	    cout << "Performing structure-only BA:"   << endl;
	    g2o::OptimizableGraph::VertexContainer points;
	    for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) 
	    {
	      g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
	      if (v->dimension() == 3)
	        points.push_back(v);
	    }   
	    structure_only_ba.calc(points, 100000);
	}

    //所有特征点的位置
    for (int i=0; i<n_points; i++)
    {
        g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+n_poses));
        cout<<"vertex id "<<i+n_poses<<", pos = ";
        Eigen::Vector3d pos = v->estimate();
        cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
    }




    // optimizer.optimize(1);
    // cout<<"优化完毕"<<endl;


    // //所有特征点的位置
    // for (int i=0; i<n_points; i++)
    // {
    //     g2o::VertexSBAPointXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+n_poses));
    //     cout<<"vertex id "<<i+n_poses<<", pos = ";
    //     Eigen::Vector3d pos = v->estimate();
    //     cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
    // }
}