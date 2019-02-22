#include "mapping.h"
#include "utils.h"

#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace cv;

Mapping::Mapping()
{
	cout<<"Construct Mapping"<<endl;

}


void Mapping::CreateMapPoints()
{








}


void Mapping::MovingObjectTest()
{

	cout<<"MovingObjectTest"<<endl;
	cv::Mat K, DistCoef;
	int width, height;
	GetParameter(K, DistCoef, width, height);

    int n_data = 150;
    vector<Mat> T_w_cs;
    vector<string> imagenames;
    Utils::readCsv("../data_movingtest/movingtest-vins_estimator-camera_pose.csv", n_data, T_w_cs, imagenames);

    //ref
    Mat ref = imread("../data_movingtest/imgs/"+imagenames[121]+".png", 0);    //gray_scale
    Mat refUndistorted;
    undistort(ref, refUndistorted, K, DistCoef);
    Mat T_w_ref = T_w_cs[121];
 
    //cur
    Mat cur = imread("../data_movingtest/imgs/"+imagenames[131]+".png", 0);    //gray_scale
    Mat curUndistorted;
    undistort(cur, curUndistorted, K, DistCoef);
    Mat T_w_cur = T_w_cs[131];

   //draw ref, cur
    Mat plot_ref, plot_cur;        
    cvtColor(refUndistorted, plot_ref, CV_GRAY2BGR);
    cvtColor(curUndistorted, plot_cur, CV_GRAY2BGR);

    //init frames
    Frame* frame1 = new Frame(refUndistorted, K, DistCoef);
    frame1->SetPose(T_w_ref.inv());

    Frame* frame2 = new Frame(curUndistorted, K, DistCoef);
    frame2->SetPose(T_w_cur.inv());

    //get epipole
    Point2d epipole = GetEpipole(frame1, frame2);
    cv::circle(plot_cur, epipole, 5, Scalar(255,255,0), -1);


    //testing points
    vector<Point2d> pts;
    pts.push_back(Point2d(600,130));
    //pts.push_back(Point2d(866,166));
    pts.push_back(Point2d(693,530));
    pts.push_back(Point2d(681,227));

    for(int i=0; i<pts.size(); i++)
    {
        cv::circle(plot_ref, pts[i], 3, Scalar(255,0,0), -1);
        Point3d line = GetEpipolarLine(frame1, frame2, pts[i]);

        double a,b,c;

        a = line.x;
        b = line.y;
        c = line.z;
        cv::line(plot_cur, Point(0,-c/b), Point(width, -(c+a*width)/b), Scalar(0,0,255));
    }

   
    imshow("plot_ref", plot_ref);
    imshow("plot_cur", plot_cur);
	waitKey(0);
}


/*
private
*/
// |xp2  - p0 |     |0|
// |yp2  - p1 | X = |0| ===> AX = 0
// |x'p2'- p0'|     |0|
// |y'p2'- p1'|     |0|
bool Mapping::Triangulation(const Mat& Tcw1, const Mat& Tcw2, const Point3d& pt_cam1, const Point3d& pt_cam2, Point3d& x3Dp)
{


    cv::Mat A(4,4, CV_64F);
    A.row(0) = pt_cam1.x*Tcw1.row(2)-Tcw1.row(0);
    A.row(1) = pt_cam1.y*Tcw1.row(2)-Tcw1.row(1);
    A.row(2) = pt_cam2.x*Tcw2.row(2)-Tcw2.row(0);
    A.row(3) = pt_cam2.y*Tcw2.row(2)-Tcw2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    cv::Mat x3D = vt.row(3).t();


    if(x3D.at<double>(3) == 0)
        return false;
    x3D = x3D.rowRange(0,3)/x3D.at<double>(3);  
    
    // check triangulation in front of camera.
    cv::Mat Rcw1, tcw1;
   	Tcw1.rowRange(0,3).colRange(0,3).copyTo(Rcw1);
    Tcw1.rowRange(0,3).col(3).copyTo(tcw1);
    
    cv::Mat Rcw2, tcw2;
   	Tcw2.colRange(0,3).copyTo(Rcw2);
    Tcw2.col(3).copyTo(tcw2);

    cv::Mat x3Dt = x3D.t();

    double z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<double>(2);
    if(z1<=0)
        return false;
        
    double z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<double>(2);
    if(z2<=0)
        return false;

    x3Dp = Point3d(x3D.at<double>(0), x3D.at<double>(1), x3D.at<double>(2));

}

cv::Mat Mapping::ComputeF21(Frame* frame1, Frame* frame2)
{

	Mat Tcw1 = frame1->GetPose();
	Mat Tcw2 = frame2->GetPose();
	Mat T21 = Tcw2*Tcw1.inv();

    Mat R21, t21;
   	T21.rowRange(0,3).colRange(0,3).copyTo(R21);
    T21.rowRange(0,3).col(3).copyTo(t21);
    
    Mat t21x = ( Mat_<double> ( 3,3 ) <<
                0,                      -t21.at<double> (2),   t21.at<double> (1),
                t21.at<double> (2),     0,                     -t21.at<double> (0),
                -t21.at<double> (1),    t21.at<double> (0),    0);

    Mat E = t21x * R21;

    Mat K1 = frame1->mK;
    Mat K2 = frame2->mK;
    Mat F = K2.inv().t() * E * K1.inv();

    //clean up F
    Eigen::MatrixXf f(3,3);
    f(0,0) = F.at<double>(0,0); f(0,1) = F.at<double>(0,1); f(0,2) = F.at<double>(0,2);
    f(1,0) = F.at<double>(1,0); f(1,1) = F.at<double>(1,1); f(1,2) = F.at<double>(1,2);
    f(2,0) = F.at<double>(2,0); f(2,1) = F.at<double>(2,1); f(2,2) = F.at<double>(2,2);

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(f, Eigen::ComputeFullV | Eigen::ComputeFullU); // ComputeThinU | ComputeThinV
    Eigen::MatrixXf singular_values = svd.singularValues();
    Eigen::MatrixXf left_singular_vectors = svd.matrixU();
    Eigen::MatrixXf right_singular_vectors = svd.matrixV();
    
    Eigen::MatrixXf d(3,3);
    d(0,0) = singular_values(0); d(0,1) = 0;                  d(0,2) = 0;
    d(1,0) = 0;                  d(1,1) = singular_values(1); d(1,2) = 0;
    d(2,0) = 0;                  d(2,1) = 0;                  d(2,2) = 0;

    f = left_singular_vectors*d*right_singular_vectors.transpose();
    F = ( Mat_<double> ( 3,3 ) << f(0,0), f(0,1), f(0,2), f(1,0), f(1,1), f(1,2), f(2,0), f(2,1), f(2,2));

    return F;
}

cv::Point2d Mapping::GetEpipole(Frame* frame1, Frame* frame2)
{

	Mat Rcw2 = frame2->GetRotation();
	Mat tcw2 = frame2->GetTranslation();

	Mat Ow1 = frame1->GetCameraCenter();
	Mat Oc2w1 = Rcw2 * Ow1 + tcw2;

	double fx = frame1->fx;
	double fy = frame1->fy;
	double cx = frame1->cx;
	double cy = frame1->cy;

	return Point2d(Oc2w1.at<double>(0)*fx/Oc2w1.at<double>(2) + cx, Oc2w1.at<double>(1)*fy/Oc2w1.at<double>(2) + cy);
}

cv::Point3d Mapping::GetEpipolarLine(Frame* frame1, Frame* frame2, const cv::Point2d& pt1)
{

    Mat F21 = ComputeF21(frame1, frame2);
	Mat pt1_ = ( Mat_<double> ( 3,1 ) <<  pt1.x, pt1.y, 1 );
	Mat line = F21 * pt1_;
    
    return Point3d(line.at<double>(0,0), line.at<double>(1,0), line.at<double>(2,0));
}

void Mapping::GetParameter(cv::Mat& K, cv::Mat& DistCoef, int& width, int& height)
{
    width = 1280;
    height = 720;
    K = ( Mat_<double> ( 3,3 ) << 712.58465576171875000, 0, 613.71289062500000000, 0, 713.57849121093750000, 386.50469970703125000, 0, 0, 1 );
    DistCoef= ( Mat_<double> ( 1,4 ) << -0.29970550537109375, 0.07715606689453125, -0.00035858154296875, 0.00141906738281250);
}

