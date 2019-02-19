#include "test.h"
#include <iostream>

using namespace std;

Test::Test()
{
	cout<<"Construct Test Senario"<<endl;
}

// void Test::testMovingObject()
// {

// 	cout<<"testMovingObject"<<endl;


//     //mynteye parameter
//     int width = 1280;
//     int height = 720;
//     Mat K = ( Mat_<double> ( 3,3 ) << 712.58465576171875000, 0, 613.71289062500000000, 0, 713.57849121093750000, 386.50469970703125000, 0, 0, 1 );
//     Mat distCoeffs= ( Mat_<double> ( 1,4 ) << -0.29970550537109375, 0.07715606689453125, -0.00035858154296875, 0.00141906738281250);


//     //construct
//     Utils *utils = new Utils();
//     Mapping *mapping = new Mapping(K);


//     int n_data = 150;
//     int ref_index = 121;
//     int cur_index = 131;
//     vector<Mat> T_w_cs;
//     vector<string> imagenames;
//     utils->readCsv("../data_movingtest/movingtest-vins_estimator-camera_pose.csv", n_data, T_w_cs, imagenames);


//     //reference image
//     Mat ref = imread("../data_movingtest/imgs/"+imagenames[ref_index]+".png", 0);    //gray_scale
//     Mat refUndistorted;
//     undistort(ref, refUndistorted, K, distCoeffs);
//     Mat T_w_ref = T_w_cs[ref_index];

    
//     //reference image
//     Mat cur = imread("../data_movingtest/imgs/"+imagenames[cur_index]+".png", 0);    //gray_scale
//     Mat curUndistorted;
//     undistort(cur, curUndistorted, K, distCoeffs);
//     Mat T_w_cur = T_w_cs[cur_index];


//    //draw ref, cur
//     Mat plot_ref, plot_cur;        
//     cvtColor(refUndistorted, plot_ref, CV_GRAY2BGR);
//     cvtColor(curUndistorted, plot_cur, CV_GRAY2BGR);
    
 
//     //get epipole
//     Point2d epipole;
//     mapping->getEpipole(T_w_ref, T_w_cur, epipole);
//     cv::circle(plot_cur, epipole, 5, Scalar(255,255,0), -1);

//     //testing points
//     vector<Point2d> pts;
//     pts.push_back(Point2d(600,130));
//     //pts.push_back(Point2d(866,166));
//     pts.push_back(Point2d(693,530));
//     pts.push_back(Point2d(681,227));

//     for(int i=0; i<pts.size(); i++)
//     {
//         cv::circle(plot_ref, pts[i], 3, Scalar(255,0,0), -1);
//         Point3d line_cur;
//         mapping->getEpipolarLine(pts[i], T_w_ref, T_w_cur, line_cur);    

//         double a,b,c;

//         a = line_cur.x;
//         b = line_cur.y;
//         c = line_cur.z;
//         cv::line(plot_cur, Point(0,-c/b), Point(width, -(c+a*width)/b), Scalar(0,0,255));
//     }

//     imshow("plot_ref", plot_ref);
//     imshow("plot_cur", plot_cur);
//     waitKey(0);
// }

// void Test::testReconstructMonitor()
// {

// 	cout<<"testReconstructMonitor"<<endl;


// 	//mynteye parameter
// 	int width = 1280;
//     int height = 720;
//     Mat K = ( Mat_<double> ( 3,3 ) << 712.58465576171875000, 0, 613.71289062500000000, 0, 713.57849121093750000, 386.50469970703125000, 0, 0, 1 );
//     Mat distCoeffs= ( Mat_<double> ( 1,4 ) << -0.29970550537109375, 0.07715606689453125, -0.00035858154296875, 0.00141906738281250);


//     //monitor ground_truth
//     double monitor_width = 0.54;
//     double monitor_height = 0.32;
//     double monitor_cross_length = 0.63;


//     //construct
//     Utils *utils = new Utils();
//     Mapping *mapping = new Mapping(K);
//     PCL *pcl = new PCL();


//     //read images and its pose
//     int n_data = 75;
//     int n_train = 40;
//     int ref_index = 25;
//     vector<Mat> T_w_cs;
//     vector<string> imagenames;
//     utils->readCsv("../data_mydesktop_mynt_eye/mydesktop-vins_estimator-camera_pose.csv", n_data, T_w_cs, imagenames);


//     //read monitor vertex
//     std::vector<std::vector<Point2d>> vertexss;
//     utils->readMonitorVertex(vertexss);


//     //reference frame
//     Mat ref = imread("../data_mydesktop_mynt_eye/imgs/"+imagenames[ref_index]+".png", 0);    //gray_scale
//     Mat refUndistorted;
//     undistort(ref, refUndistorted, K, distCoeffs);
//     Mat T_w_ref = T_w_cs[ref_index];
    

//     //init seeds
//     double frame_mean_dpeth = 3; 
//     double frame_min_depth = 0.1;
//     std::vector<Seed> seeds;
//     mapping->initSeeds(vertexss[0], frame_mean_dpeth, frame_min_depth, seeds);


//     //draw ref
//     Mat plot_ref;        
//     cvtColor(refUndistorted, plot_ref, CV_GRAY2BGR);
//     for(int i=0; i<seeds.size(); i++)
//         cv::circle(plot_ref, seeds[i].pt, 3, Scalar(0,255,0), -1);
//     imshow("plot_ref", plot_ref);


//     //current frames
//     vector<Mat> curUndistorteds;
//     vector<Mat> T_w_curs;
//     for(int index=ref_index+1; index<(ref_index+n_train); index++)
//     {
//         Mat cur = imread("../data_mydesktop_mynt_eye/imgs/"+imagenames[index]+".png", 0);    //gray_scale
//         Mat curUndistorted;
//         undistort(cur, curUndistorted, K, distCoeffs);
//         Mat T_w_cur = T_w_cs[index];

//         curUndistorteds.push_back(curUndistorted);
//         T_w_curs.push_back(T_w_cur);
//     }


//     //update seeds
//     vector<Point3d> centers;
//     int trainning_size = curUndistorteds.size();
//     for(int img=0; img<trainning_size; img++)
//     {
//         //read image and its pose
//         Mat curUndistorted = curUndistorteds[img];
//         Mat T_w_cur = T_w_curs[img];

//         //avoid dist of two cameras not long enough 
//         if(mapping->getCentersDist(T_w_ref, T_w_cur)<0.1)
//             continue;

//         //record path
//         centers.push_back(mapping->getCurCenter(T_w_ref, T_w_cur));

//         for(int i=0; i<seeds.size(); i++)
//         {
//             //update seed 
//             Point2d pt_ref = vertexss[0][i];
//             Point2d pt_cur = vertexss[img+1][i];
//             mapping->updateSeed(T_w_ref, T_w_cur, pt_ref, pt_cur, &seeds[i]);
//         }
//     }


//     //calculate error
//     vector<double> repro_errors(seeds.size(),0.0);
//     for(int img=0; img<trainning_size; img++)
//     {
//         //read image and its pose
//         Mat curUndistorted = curUndistorteds[img];
//         Mat T_w_cur = T_w_curs[img];
        
//         Mat plot_cur;        
//         cvtColor(curUndistorted, plot_cur, CV_GRAY2BGR);
        
//         for(int i=0; i<seeds.size(); i++)
//         {
//             Point2d pt_cur = vertexss[img+1][i];
//             Point2d pt_cur_repro;
//             mapping->reproSeed(&seeds[i], T_w_ref, T_w_cur, pt_cur_repro);
//             repro_errors[i] += mapping->reproError(pt_cur, pt_cur_repro);

//             cv::circle(plot_cur, pt_cur, 3, Scalar(0,255,0), -1);
//             cv::circle(plot_cur, pt_cur_repro, 3, Scalar(0,0,255), -1);
//         }

//         imshow("plot_cur", plot_cur);
//         waitKey(0);
//     }

//     for(int i=0; i<repro_errors.size(); i++)
//     {
//         repro_errors[i] /= trainning_size;
//         cout<<"average reprojection error "<<i<<" : "<<repro_errors[i]<<endl;
//     }



//     //vertex 3D points
//     vector<Point3d> pts_vertex3D;
//     for(int i=0; i<seeds.size(); i++)
//     	pts_vertex3D.push_back(seeds[i].f * (1/seeds[i].mu));
    

//     //calculate monotor width and height
//     cout<<"width_up_error: "<<(norm(pts_vertex3D[0]-pts_vertex3D[1])-monitor_width)<<endl;
//     cout<<"width_bottom_error: "<<(norm(pts_vertex3D[2]-pts_vertex3D[3])-monitor_width)<<endl;
//     cout<<"height_left_error: "<<(norm(pts_vertex3D[0]-pts_vertex3D[2])-monitor_height)<<endl;
//     cout<<"height_right_error: "<<(norm(pts_vertex3D[1]-pts_vertex3D[3])-monitor_height)<<endl;
//     cout<<"cross_error: "<<(norm(pts_vertex3D[0]-pts_vertex3D[3])-monitor_cross_length)<<endl;


//     //PCA optimization 
//     Point3d normal_vector; 
//     double d;
//     mapping->findPlane(pts_vertex3D, normal_vector, d);

//     Point3d camera_center = Point3d(0.0,0.0,0.0);
//     vector<Point3d> fs;
//     for(int i=0; i<seeds.size(); i++)
//         fs.push_back(seeds[i].f);
//     vector<Point3d> intersect_points;
//     mapping->calPlaneLinesIntersectPoints(camera_center, fs, normal_vector, d, intersect_points);

//     //calculate monotor width and height
//     cout<<"width_up_error_pca: "<<(norm(intersect_points[0]-intersect_points[1])-monitor_width)<<endl;
//     cout<<"width_bottom_error_pca: "<<(norm(intersect_points[2]-intersect_points[3])-monitor_width)<<endl;
//     cout<<"height_left_error_pca: "<<(norm(intersect_points[0]-intersect_points[2])-monitor_height)<<endl;
//     cout<<"height_right_error_pca: "<<(norm(intersect_points[1]-intersect_points[3])-monitor_height)<<endl;
//     cout<<"cross_error_pca: "<<(norm(intersect_points[0]-intersect_points[3])-monitor_cross_length)<<endl;


//     //update mu
//     for(int i=0; i<seeds.size(); i++)
//     {
//         double z = norm(intersect_points[i]);
//         seeds[i].mu = 1/z;
//     }




// 	//VISUALIZE
//     //monitor
//     std::vector<cv::Point3d> pts3D;
//     std::vector<cv::Vec3b> pts_color;
//     //vertex
//     for(int i=0; i<pts_vertex3D.size(); i++)
//     {
//         pts3D.push_back(pts_vertex3D[i]);
//         pts_color.push_back(Vec3b(255,255,255));
//     }
//     for(int i=0; i<intersect_points.size(); i++)
//     {
//         pts3D.push_back(intersect_points[i]);
//         pts_color.push_back(Vec3b(255,255,0));
//     }
//     //path
//     for(int i=0; i<centers.size(); i++)
//     {
//         pts3D.push_back(centers[i]);
//         pts_color.push_back(Vec3b(0,255,0));
//     }

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr = pcl->generatePointCloudColor(pts3D, pts_color);
//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = pcl->rgbVis(point_cloud_ptr);
//     //--------------------
//     // -----Main loop-----
//     //--------------------
//     while (!viewer->wasStopped ())
//     {
//         viewer->spinOnce (100);
//         boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//     }

// }


// void Test::testReconstructMonitorHomography()
// {

// 	cout<<"testReconstructMonitorHomography"<<endl;


// 	//mynteye parameter
// 	int width = 1280;
//     int height = 720;
//     Mat K = ( Mat_<double> ( 3,3 ) << 712.58465576171875000, 0, 613.71289062500000000, 0, 713.57849121093750000, 386.50469970703125000, 0, 0, 1 );
//     Mat distCoeffs= ( Mat_<double> ( 1,4 ) << -0.29970550537109375, 0.07715606689453125, -0.00035858154296875, 0.00141906738281250);


//     //monitor ground_truth
//     double monitor_width = 0.54;
//     double monitor_height = 0.32;
//     double monitor_cross_length = 0.63;


//     //construct
//     Utils *utils = new Utils();
//     Mapping *mapping = new Mapping(K);
//     PCL *pcl = new PCL();


//     //read images and its pose
//     int n_data = 75;
//     int n_train = 40;
//     int ref_index = 25;
//     vector<Mat> T_w_cs;
//     vector<string> imagenames;
//     utils->readCsv("../data_mydesktop_mynt_eye/mydesktop-vins_estimator-camera_pose.csv", n_data, T_w_cs, imagenames);


//     //read monitor vertex
//     std::vector<std::vector<Point2d>> vertexss;
//     utils->readMonitorVertex(vertexss);


//     //reference frame
//     Mat ref = imread("../data_mydesktop_mynt_eye/imgs/"+imagenames[ref_index]+".png", 0);    //gray_scale
//     Mat refUndistorted;
//     undistort(ref, refUndistorted, K, distCoeffs);
//     Mat T_w_ref = T_w_cs[ref_index];


//     //boundry
//     int x1 = max(vertexss[0][0].x, vertexss[0][2].x);
//     int x2 = min(vertexss[0][1].x, vertexss[0][3].x);
//     int y1 = max(vertexss[0][0].y, vertexss[0][1].y);
//     int y2 = min(vertexss[0][2].y, vertexss[0][3].y);

//     //seeds on plane, needed to be inside the monitor
//     vector<Point2d> pts_ref;
//     for(int i=x1+10; i<=x2-10; i+=20)
//     {
//     	for(int j=y1+10; j<=y2-10; j+=20)
//     		pts_ref.push_back(Point2d(i,j));
//     }


//     //init seeds
//     double frame_mean_dpeth = 3; 
//     double frame_min_depth = 0.1;
//     std::vector<Seed> seeds_vertex, seeds_plane;
//     mapping->initSeeds(vertexss[0], frame_mean_dpeth, frame_min_depth, seeds_vertex);
//     mapping->initSeeds(pts_ref, frame_mean_dpeth, frame_min_depth, seeds_plane);


//     //draw ref
//     Mat plot_ref;        
//     cvtColor(refUndistorted, plot_ref, CV_GRAY2BGR);    
//     for(int i=0; i<seeds_vertex.size(); i++)
//         cv::circle(plot_ref, seeds_vertex[i].pt, 3, Scalar(0,255,0), -1);
//     for(int i=0; i<seeds_plane.size(); i++)
//         cv::circle(plot_ref, seeds_plane[i].pt, 3, Scalar(255,255,0), -1);    
//     imshow("plot_ref", plot_ref);


//     //current frames
//     vector<Mat> curUndistorteds;
//     vector<Mat> T_w_curs;
//     for(int index=ref_index+1; index<(ref_index+n_train); index++)
//     {
//         Mat cur = imread("../data_mydesktop_mynt_eye/imgs/"+imagenames[index]+".png", 0);    //gray_scale
//         Mat curUndistorted;
//         undistort(cur, curUndistorted, K, distCoeffs);
//         Mat T_w_cur = T_w_cs[index];

//         curUndistorteds.push_back(curUndistorted);
//         T_w_curs.push_back(T_w_cur);
//     }


//     //generate current plane points 
//     vector<vector<Point2d>> pts_curs;
//     int trainning_size = curUndistorteds.size();
//     for(int img=0; img<trainning_size; img++)
//     {
//     	//read image and its pose
//         Mat curUndistorted = curUndistorteds[img];
//         Mat T_w_cur = T_w_curs[img];

//         //find homography
//         vector<Point2d> ref_vertexs = vertexss[0];
//         vector<Point2d> cur_vertexs = vertexss[img+1];
//         Mat H = mapping->computeH(ref_vertexs, cur_vertexs);

//         //perspectiveTransform plane points
// 		vector<Point2d> pts_dst;
//         mapping->perspectiveTransform(H, pts_ref, pts_dst);

//         pts_curs.push_back(pts_dst);
//     }


//     //update seeds
//     vector<Point3d> centers;
//     for(int img=0; img<trainning_size; img++)
//     {
//         //read image and its pose
//         Mat curUndistorted = curUndistorteds[img];
//         Mat T_w_cur = T_w_curs[img];

//         //avoid dist of two cameras not long enough 
//         if(mapping->getCentersDist(T_w_ref, T_w_cur)<0.1)
//             continue;

//         //record path
//         centers.push_back(mapping->getCurCenter(T_w_ref, T_w_cur));

//         //update vertex seeds
//         for(int i=0; i<seeds_vertex.size(); i++)
//         {
//             //update seed 
//             Point2d pt_ref = seeds_vertex[i].pt;
//             Point2d pt_cur = vertexss[img+1][i];
//             mapping->updateSeed(T_w_ref, T_w_cur, pt_ref, pt_cur, &seeds_vertex[i]);
//         }

//         //update plane seeds
//         for(int i=0; i<seeds_plane.size(); i++)
//         {
//             //update seed 
//             Point2d pt_ref = seeds_plane[i].pt;
//             Point2d pt_cur = pts_curs[img][i];
//             mapping->updateSeed(T_w_ref, T_w_cur, pt_ref, pt_cur, &seeds_plane[i]);
// 		}
//     }


//     //calculate error
//     vector<double> repro_vertex_errors(seeds_vertex.size(),0.0);
//     vector<double> repro_plane_errors(seeds_plane.size(),0.0);
//     for(int img=0; img<trainning_size; img++)
//     {
//         //read image and its pose
//         Mat curUndistorted = curUndistorteds[img];
//         Mat T_w_cur = T_w_curs[img];
        
//         Mat plot_cur;        
//         cvtColor(curUndistorted, plot_cur, CV_GRAY2BGR);
        
//         for(int i=0; i<seeds_vertex.size(); i++)
//         {
//             Point2d pt_cur = vertexss[img+1][i];
//             Point2d pt_cur_repro;
//             mapping->reproSeed(&seeds_vertex[i], T_w_ref, T_w_cur, pt_cur_repro);
//             repro_vertex_errors[i] += mapping->reproError(pt_cur, pt_cur_repro);

//             cv::circle(plot_cur, pt_cur, 3, Scalar(0,255,0), -1);
//             cv::circle(plot_cur, pt_cur_repro, 3, Scalar(0,0,255), -1);
//         }

//         for(int i=0; i<seeds_plane.size(); i++)
//         {
//         	Point2d pt_cur = pts_curs[img][i];
//         	Point2d pt_cur_repro;
//         	mapping->reproSeed(&seeds_plane[i], T_w_ref, T_w_cur, pt_cur_repro);
//         	repro_plane_errors[i] += mapping->reproError(pt_cur, pt_cur_repro);

//             cv::circle(plot_cur, pt_cur, 3, Scalar(0,255,0), -1);
//             cv::circle(plot_cur, pt_cur_repro, 3, Scalar(0,0,255), -1);
// 		}


//         imshow("plot_cur", plot_cur);
//         waitKey(0);
//     }

//     for(int i=0; i<repro_vertex_errors.size(); i++)
//     {
//         repro_vertex_errors[i] /= trainning_size;
//         cout<<"average reprojection vertex error "<<i<<" : "<<repro_vertex_errors[i]<<endl;
//     }

//     double average_plane_error = 0.0;
//     for(int i=0; i<repro_plane_errors.size(); i++)
//     {
//         repro_plane_errors[i] /= trainning_size;
//         average_plane_error += repro_plane_errors[i];
//     }
//     average_plane_error /= repro_plane_errors.size();
//     cout<<"average reprojection plane error : "<<average_plane_error<<endl;



//     //VISUALIZE
//     std::vector<cv::Point3d> pts3D;
//     std::vector<cv::Vec3b> pts_color;
//     for(int i=0; i<seeds_vertex.size(); i++)
//     {
//         pts3D.push_back(seeds_vertex[i].f * (1/seeds_vertex[i].mu));
//         pts_color.push_back(Vec3b(255,255,255));
//     }
//     for(int i=0; i<seeds_plane.size(); i++)
//     {
//         pts3D.push_back(seeds_plane[i].f * (1/seeds_plane[i].mu));
//         pts_color.push_back(Vec3b(255,255,0));
//     }
//     //path
//     for(int i=0; i<centers.size(); i++)
//     {
//         pts3D.push_back(centers[i]);
//         pts_color.push_back(Vec3b(0,255,0));
//     }

//     //calculate monotor width and height
//     cout<<"width_up_error: "<<(norm(pts3D[0]-pts3D[1])-monitor_width)<<endl;
//     cout<<"width_bottom_error: "<<(norm(pts3D[2]-pts3D[3])-monitor_width)<<endl;
//     cout<<"height_left_error: "<<(norm(pts3D[0]-pts3D[2])-monitor_height)<<endl;
//     cout<<"height_right_error: "<<(norm(pts3D[1]-pts3D[3])-monitor_height)<<endl;
//     cout<<"cross_error: "<<(norm(pts3D[0]-pts3D[3])-monitor_cross_length)<<endl;

//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr = pcl->generatePointCloudColor(pts3D, pts_color);
//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = pcl->rgbVis(point_cloud_ptr);
//     //--------------------
//     // -----Main loop-----
//     //--------------------
//     while (!viewer->wasStopped ())
//     {
//         viewer->spinOnce (100);
//         boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//     }
// }

void Test::testReconstructLandmark()
{

    cout<<"testReconstructLandmark"<<endl;


    //mynteye parameter
    Mat K, distort; 
    int width, height;
    getMyntParameter(K, distort, width, height);


    //landmark ground_truth
    double landmark_width = 0.177;
    double landmark_height = 0.120;

    
    //construct
    Utils *utils = new Utils();
    Mapping *mapping = new Mapping(K);
    PCL *pcl = new PCL();


    //read images and its pose
    int n_data = 75;
    vector<Mat> T_w_cs_;
    vector<string> imagenames;
    utils->readCsv("../data_landmark_color/landmark-vins_estimator-camera_pose.csv", n_data, T_w_cs_, imagenames);


    //get we need, img and its pose
    vector<Mat> imgsUndistorted;
    vector<Mat> T_w_cs;
    for(int i=18; i<54; i+=2)
    {
        Mat img = imread("../data_landmark_color/imgs/"+imagenames[i]+".png", 1);    //color
        Mat imgUndistorted;
        undistort(img, imgUndistorted, K, distort);
        imgsUndistorted.push_back(imgUndistorted);
        T_w_cs.push_back(T_w_cs_[i]);
    }

    int n_train = imgsUndistorted.size();


    //vertexss
    std::vector<std::vector<Point2d>> vertexss;
    utils->readLandMarkVertex(vertexss);



    //reference frame
    Mat refUndistorted = imgsUndistorted[0];
    Mat T_w_ref = T_w_cs[0];


    //landmark boundry
    int x1 = max(vertexss[0][0].x, vertexss[0][2].x);
    int x2 = min(vertexss[0][1].x, vertexss[0][3].x);
    int y1 = max(vertexss[0][0].y, vertexss[0][1].y);
    int y2 = min(vertexss[0][2].y, vertexss[0][3].y);


    //seeds on plane, needed to be inside the monitor
    vector<Point2d> pts_ref;
    for(int i=x1+5; i<=x2-5; i+=1)
    {
        for(int j=y1+5; j<=y2-5; j+=1)
            pts_ref.push_back(Point2d(i,j));
    }

    //color
    vector<Vec3b> colors_ref;
    for(int i=0; i<pts_ref.size(); i++)
        colors_ref.push_back(refUndistorted.at<Vec3b>(pts_ref[i].y, pts_ref[i].x));  
    


    //init seeds
    double frame_mean_dpeth = 3; 
    double frame_min_depth = 0.1;
    std::vector<Seed> seeds_vertex, seeds_plane;
    mapping->initSeeds(vertexss[0], frame_mean_dpeth, frame_min_depth, seeds_vertex);
    mapping->initSeeds(pts_ref, frame_mean_dpeth, frame_min_depth, seeds_plane);


    //draw ref
    Mat plot_ref = refUndistorted.clone();        
    for(int i=0; i<seeds_vertex.size(); i++)
        cv::circle(plot_ref, seeds_vertex[i].pt, 2, Scalar(255,0,0), -1);
    for(int i=0; i<seeds_plane.size(); i++)
        cv::circle(plot_ref, seeds_plane[i].pt, 2, Scalar(255,255,0), -1);
    imshow("plot_ref", plot_ref);
    


    //generate current plane points
    vector<vector<Point2d>> pts_curs;
    for(int i=1; i<n_train; i++)
    {
        //find homography
        vector<Point2d> ref_vertexs = vertexss[0];
        vector<Point2d> cur_vertexs = vertexss[i];
        Mat H = mapping->computeH(ref_vertexs, cur_vertexs);

        //perspectiveTransform plane points
        vector<Point2d> pts_dst;
        mapping->perspectiveTransform(H, pts_ref, pts_dst);
        pts_curs.push_back(pts_dst);
    }



    //update seeds
    vector<Point3d> centers;
    for(int i=1; i<n_train; i++)
    {
        Mat curUndistorted = imgsUndistorted[i];
        Mat T_w_cur = T_w_cs[i];

        //record path
        centers.push_back(mapping->getCurCenter(T_w_ref, T_w_cur));

        //avoid dist of two cameras not long enough 
        if(mapping->getCentersDist(T_w_ref, T_w_cur)<0.1)
            continue;

        //update vertex seeds
        for(int j=0; j<seeds_vertex.size(); j++)
        {
            //update seed 
            Point2d pt_ref = seeds_vertex[j].pt;
            Point2d pt_cur = vertexss[i][j];
            mapping->updateSeed(T_w_ref, T_w_cur, pt_ref, pt_cur, seeds_vertex[j]);
        }

        //update plane seeds
        for(int j=0; j<seeds_plane.size(); j++)
        {
            //update seed 
            Point2d pt_ref = seeds_plane[j].pt;
            Point2d pt_cur = pts_curs[i-1][j];
            mapping->updateSeed(T_w_ref, T_w_cur, pt_ref, pt_cur, seeds_plane[j]);
        }
    }










    //calculate reprojection error of vertexs and plane
    vector<double> repro_vertex_errors(seeds_vertex.size(),0.0); 
    vector<double> repro_plane_errors(seeds_plane.size(),0.0);
    for(int i=1; i<n_train; i++)
    {

        Mat curUndistorted = imgsUndistorted[i];
        Mat T_w_cur = T_w_cs[i];

        Mat plot_cur = curUndistorted.clone();        

        for(int j=0; j<seeds_vertex.size(); j++)
        {
            Point2d pt_cur = vertexss[i][j];
            Point2d pt_cur_repro;
            mapping->reproSeed(seeds_vertex[j], T_w_ref, T_w_cur, pt_cur_repro);
            repro_vertex_errors[j] += mapping->reproError(pt_cur, pt_cur_repro);

            cv::circle(plot_cur, pt_cur, 3, Scalar(0,255,0), -1);
            cv::circle(plot_cur, pt_cur_repro, 3, Scalar(0,0,255), -1);
        }

        for(int j=0; j<seeds_plane.size(); j++)
        {
            Point2d pt_cur = pts_curs[i-1][j];
            Point2d pt_cur_repro;
            mapping->reproSeed(seeds_plane[j], T_w_ref, T_w_cur, pt_cur_repro);
            repro_plane_errors[j] += mapping->reproError(pt_cur, pt_cur_repro);

            cv::circle(plot_cur, pt_cur, 3, Scalar(0,255,0), -1);
            cv::circle(plot_cur, pt_cur_repro, 3, Scalar(0,0,255), -1);
        }

        imshow("plot_cur", plot_cur);
        waitKey(0);
    }

    double average_vertex_error = 0.0;
    for(int i=0; i<repro_vertex_errors.size(); i++)
    {
        repro_vertex_errors[i] /= (n_train-1);
        cout<<"average reprojection each vertex error "<<i<<" : "<<repro_vertex_errors[i]<<endl;
        average_vertex_error += repro_vertex_errors[i];
    }
    average_vertex_error /= repro_vertex_errors.size();
    cout<<"average reprojection vertex error:  "<<average_vertex_error<<endl;

    double average_plane_error = 0.0;
    for(int i=0; i<repro_plane_errors.size(); i++)
    {
        repro_plane_errors[i] /= (n_train-1);
        average_plane_error += repro_plane_errors[i];
    }
    average_plane_error /= repro_plane_errors.size();
    cout<<"average reprojection plane error : "<<average_plane_error<<endl;













    //3D points of vertex
    std::vector<cv::Point3f> object3Dpnts;
    for(int i=0; i<seeds_vertex.size(); i++)
        object3Dpnts.push_back(seeds_vertex[i].f * (1/seeds_vertex[i].mu));


    //ground truth error
    cout<<"width_up_error_rate: "<<(norm(object3Dpnts[0]-object3Dpnts[1])-landmark_width)/landmark_width*100<<" %"<<endl;
    cout<<"width_bottom_error_rate: "<<(norm(object3Dpnts[2]-object3Dpnts[3])-landmark_width)/landmark_width*100<<" %"<<endl;
    cout<<"height_left_error_rate: "<<(norm(object3Dpnts[0]-object3Dpnts[2])-landmark_height)/landmark_height*100<<" %"<<endl;
    cout<<"height_right_error_rate: "<<(norm(object3Dpnts[1]-object3Dpnts[3])-landmark_height)/landmark_height*100<<" %"<<endl;


    //Localization between PNP
    vector<Point3d> re_centers;
    for(int i=1; i<n_train; i++)
    {
        std::vector<cv::Point2f> imgpnts;
        for(int j=0; j<seeds_vertex.size(); j++)
            imgpnts.push_back(vertexss[i][j]);

        Mat rvec, R, tvec;
        Mat _distort= ( Mat_<double> ( 1,4 ) << 0.0, 0.0, 0.0, 0.0);

        solvePnPRansac(object3Dpnts, imgpnts, K, _distort, rvec, tvec, false, CV_P3P);
        Rodrigues(rvec, R);

        Mat R_ref_cur = R.t();
        Mat t_ref_cur = -R_ref_cur*tvec;
        re_centers.push_back(Point3d(t_ref_cur.at<double>(0), t_ref_cur.at<double>(1), t_ref_cur.at<double>(2)));
    }

    //PNP result error
    double average_centers_error = 0.0;
    for(int i=0; i<centers.size(); i++)
       average_centers_error += norm(centers[i]-re_centers[i]);
    average_centers_error /= centers.size();
    cout<<"average centers error: "<<average_centers_error<<endl;







































    //VISUALIZE
    std::vector<cv::Point3d> pts3D;
    std::vector<cv::Vec3b> pts_color;
    for(int i=0; i<seeds_vertex.size(); i++)
    {
        pts3D.push_back(seeds_vertex[i].f * (1/seeds_vertex[i].mu));
        pts_color.push_back(Vec3b(255,255,255));
    }
    for(int i=0; i<seeds_plane.size(); i++)
    {
        pts3D.push_back(seeds_plane[i].f * (1/seeds_plane[i].mu));
        pts_color.push_back(colors_ref[i]);
    }
    //path
    for(int i=0; i<centers.size(); i++)
    {
        pts3D.push_back(centers[i]);
        pts_color.push_back(Vec3b(0,255,0));
    }
    //re-path
    for(int i=0; i<re_centers.size(); i++)
    {
        pts3D.push_back(re_centers[i]);
        pts_color.push_back(Vec3b(0,0,255));
    }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr = pcl->generatePointCloudColor(pts3D, pts_color);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = pcl->rgbVis(point_cloud_ptr);
    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }









}



void Test::getMyntParameter(Mat& K, Mat& distort, int& width, int& height)
{

    width = 1280;
    height = 720;
    K = ( Mat_<double> ( 3,3 ) << 712.58465576171875000, 0, 613.71289062500000000, 0, 713.57849121093750000, 386.50469970703125000, 0, 0, 1 );
    distort= ( Mat_<double> ( 1,4 ) << -0.29970550537109375, 0.07715606689453125, -0.00035858154296875, 0.00141906738281250);

}
