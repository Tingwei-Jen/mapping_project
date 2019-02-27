#include "converter.h"
#include "depth_filter.h"
#include "mapping.h"
#include "map.h"
#include "optimizer.h"
#include "pcl.h"
#include "trafficsigns.h"
#include "utils.h"

#include <Eigen/Dense>
#include <iostream>
#include "opencv2/xfeatures2d.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

Mapping::Mapping()
{
	cout<<"Construct Mapping"<<endl;

}

void Mapping::TestBA()
{   
    cout<<"TestBA"<<endl;
    Mat K, DistCoef;
    int width, height;
    GetParameter(K, DistCoef, width, height);

    int n_data = 75;
    vector<Mat> T_w_cs;
    vector<string> imagenames;
    Utils::readCsv("../data_landmark_color/landmark-vins_estimator-camera_pose.csv", n_data, T_w_cs, imagenames);

    vector<Mat> imgs;
    vector<Mat> Tcws;
    for(int i=18; i<54; i+=2)
    {
        imgs.push_back(imread("../data_landmark_color/imgs/"+imagenames[i]+".png", 1));
        Tcws.push_back(T_w_cs[i].inv());
    }
    int n_train = imgs.size();


    Mat img1 = imgs[0];
    Mat img2 = imgs[5];

    Mat img1Undistorted, img2Undistorted;
    undistort(img1, img1Undistorted, K, DistCoef);
    undistort(img2, img2Undistorted, K, DistCoef);

    vector<Point2f> pts1_, pts2_; 
    if(findCorresponding(img1Undistorted, img2Undistorted, pts1_, pts2_) == false)
    {
        cout<<"匹配点不够！"<<endl;
    }

    cout<<"找到了"<<pts1_.size()<<"组对应特征点。"<<endl;

    Mat Tcw1, Tcw2;
    Tcws[0].convertTo(Tcw1, CV_32F); 
    Tcws[0].convertTo(Tcw2, CV_32F); 

    float fx = 712.58465;
    float fy = 713.57849;
    float cx = 613.71289;
    float cy = 386.50469;


    vector<Point2f> pts1, pts2; 
    int test_size = 20;

    for(int i=0; i<test_size; i++)
    {
        pts1.push_back(pts1_[i]);
        pts2.push_back(pts2_[i]);
    }

    std::vector<cv::Point3f> pts3d;
    for(int i=0; i<pts1.size(); i++)
    {
        float z = 1;
        float x = ( pts1[i].x - cx ) * z / fx; 
        float y = ( pts1[i].y - cy ) * z / fy;

        Point3f p = Point3f(x,y,z);
        pts3d.push_back(p);
    }

    //reprojection error
    double average_error = 0.0;
    int n = 0;
    Mat plot_before = img2Undistorted.clone();

    for(int i=0; i<pts3d.size(); i++)
    {
        Point3f Pw = pts3d[i];
        Mat Rcw2, tcw2;
        Rcw2 = Tcw2.rowRange(0,3).colRange(0,3);
        tcw2 = Tcw2.rowRange(0,3).col(3);
        Mat Pc_ = Rcw2*Converter::toCvMat(Pw) + tcw2;
        Point3f Pc = Converter::toCvPoint3f(Pc_);
        Point2f repro = Point2f(Pc.x*fx/Pc.z + cx, Pc.y*fy/Pc.z + cy);
        average_error += norm(pts2[i]-repro);
        n++;
        cv::circle(plot_before, pts2[i], 2, Scalar(255,0,0), -1);
        cv::circle(plot_before, repro, 2, Scalar(0,0,255), -1);
    }

    imshow("plot_before", plot_before);
    cout<<"average_error before BA: "<<average_error/n<<endl;

    Optimizer::BundleAdjustment(Tcw1, Tcw2, pts3d, pts1, pts2, 20);

    //reprojection
    average_error = 0.0;
    n = 0;
    Mat plot_after = img2Undistorted.clone();

    for(int i=0; i<pts3d.size(); i++)
    {
        Point3f Pw = pts3d[i];
        Mat Rcw2, tcw2;
        Rcw2 = Tcw2.rowRange(0,3).colRange(0,3);
        tcw2 = Tcw2.rowRange(0,3).col(3);
        Mat Pc_ = Rcw2*Converter::toCvMat(Pw) + tcw2;
        Point3f Pc = Converter::toCvPoint3f(Pc_);
        Point2f repro = Point2f(Pc.x*fx/Pc.z + cx, Pc.y*fy/Pc.z + cy);
        average_error += norm(pts2[i]-repro);
        n++;
        cv::circle(plot_after, pts2[i], 2, Scalar(255,0,0), -1);
        cv::circle(plot_after, repro, 2, Scalar(0,0,255), -1);
    }

    imshow("plot_after", plot_after);
    cout<<"average_error after BA: "<<average_error/n<<endl;

    cout<<Tcw1<<endl;
    cout<<Tcw2<<endl;



    waitKey(0);
}

void Mapping::CreateMapPoints()
{
    cout<<"CreateMapPoints"<<endl;
    Mat K, DistCoef;
    int width, height;
    GetParameter(K, DistCoef, width, height);

    int n_data = 75;
    vector<Mat> T_w_cs;
    vector<string> imagenames;
    Utils::readCsv("../data_landmark_color/landmark-vins_estimator-camera_pose.csv", n_data, T_w_cs, imagenames);

    vector<Mat> imgs;
    vector<Mat> Tcws;
    for(int i=18; i<54; i+=2)
    {
        imgs.push_back(imread("../data_landmark_color/imgs/"+imagenames[i]+".png", 1));
        Mat Twc;
        T_w_cs[i].convertTo(Twc, CV_32F);                                            //double to float
        Tcws.push_back(Twc.inv());
    }
    int n_train = imgs.size();

    vector<vector<Point2d>> verticess;
    Utils::readLandMarkVertex(verticess);

    vector<TrafficSigns*> Signss;
    for(int i=0; i<n_train; i++)
    {
        TrafficSigns* signs = new TrafficSigns();
        signs->mVertices.push_back(verticess[i][0]);
        signs->mTypes.push_back(SignVerticeType::UpLeft);
        signs->mVertices.push_back(verticess[i][1]);
        signs->mTypes.push_back(SignVerticeType::UpRight);
        signs->mVertices.push_back(verticess[i][2]);
        signs->mTypes.push_back(SignVerticeType::DownLeft);
        signs->mVertices.push_back(verticess[i][3]);
        signs->mTypes.push_back(SignVerticeType::DownRight);
        Signss.push_back(signs);
    }

    Map* map = new Map();
    int idx = 0;
    while( idx<n_train )     
    {
        if(Signss[idx]==NULL)
            continue;

        Frame* frame = new Frame(imgs[idx], K, DistCoef, Signss[idx]);                   //create frame
        frame->SetPose(Tcws[idx]);
        map->AddFrame(frame);

        if(map->GetAllSeeds().empty())                                                   //At beginning, there is no seed
        {
            
            //Mat plot_ref = frame->GetImgUndistorted().clone();
            for(int i=0; i<frame->GetTrafficSigns()->mVertices.size(); i++)
            {
                cv::Point2f pt = frame->GetTrafficSigns()->mVertices[i];
                SignVerticeType type = frame->GetTrafficSigns()->mTypes[i];

                Seed* seed = new Seed(frame, pt, type, 1.0, 0.01);                       //create seeds
                map->AddSeed(seed);
                //cv::circle(plot_ref, seed->mpt, 2, Scalar(255,0,0), -1);
            }
            // imshow("plot_ref", plot_ref);
            // waitKey(0);
        }
        else                                                                            //need to judge same features or not.
        {                                                           
            for(int i=0; i<frame->GetTrafficSigns()->mVertices.size(); i++)             //遍歷所有畫面中的角點
            {

                bool found = false;
                cv::Point2f pt = frame->GetTrafficSigns()->mVertices[i];
                SignVerticeType type = frame->GetTrafficSigns()->mTypes[i];
           
                vector<Seed*> AllSeeds = map->GetAllSeeds();                       
                for(int sd=0; sd<AllSeeds.size(); sd++)                                //與所有seed做比較
                {
                    if(type == AllSeeds[sd]->mType)
                    {                           
                        cv::Point3f line = GetEpipolarLine(AllSeeds[sd]->mframe, frame, AllSeeds[sd]->mpt);

                        if(DistPt2Line(pt,line)<=3.0)
                        {
                            //update filer
                            found = true;
                            UpdateSeed(AllSeeds[sd], frame, pt);

                            // Mat plot_cur = frame->GetImgUndistorted().clone();
                            // float a = line.x;
                            // float b = line.y;
                            // float c = line.z;
                            // cv::line(plot_cur, Point(0,-c/b), Point(width, -(c+a*width)/b), Scalar(0,0,255));
                            // cv::circle(plot_cur, pt, 2, Scalar(0,0,255), -1);
                            // imshow("plot_cur", plot_cur);
                            // waitKey(0);
                        }
                    }
                }

                if(!found)
                {
                    cout<<"create new seed"<<endl;
                    Seed* seed = new Seed(frame, pt, type, 1.0, 0.01);                   //create new seed
                    map->AddSeed(seed);
                }
            }
        }

        idx++;
    }



    //BA!!!!!!!!!!!!!!!!!!!!!!!!!!!
    vector<Frame*> Frames = map->GetAllFrames();
    vector<Seed*> Seeds = map->GetAllSeeds();

    //landmark ground_truth
    double landmark_width = 0.177;
    double landmark_height = 0.120;


    //Reprojection error
    float average_error = 0.0;
    int n = 0;
    for(int i=0; i<Frames.size(); i++)
    {
        Mat plot_cur = Frames[i]->GetImgUndistorted().clone();

        for(int j=0; j<Seeds.size(); j++)
        {
            Point2f seed_repro = Reprojection(Seeds[j], Frames[i]);
            SignVerticeType seed_type =  Seeds[j]->mType;
            
            for(int k=0; k<Frames[i]->GetTrafficSigns()->mVertices.size(); k++)         //遍歷所有畫面中的角點   
            {
                if(seed_type == Frames[i]->GetTrafficSigns()->mTypes[k])
                {
                    cv::Point2f pt = Frames[i]->GetTrafficSigns()->mVertices[k];
                    float error = norm(seed_repro-pt);
                    average_error += error;
                    n++;

                    cv::circle(plot_cur, seed_repro, 2, Scalar(0,0,255), -1);
                    cv::circle(plot_cur, pt, 2, Scalar(255,0,0), -1);
                }
            }
        }

        imshow("plot_cur", plot_cur);
        waitKey(0); 
    }

    cout<<"average vertices error before BA:  "<<average_error/n<<endl;

    vector<Point3f> pts3D_beforeBA;
    for(int i=0; i<Seeds.size(); i++)
    {
        pts3D_beforeBA.push_back(Seeds[i]->GetWorldPose());
    }


    //ground truth error
    cout<<"width_up_error_rate: "<<(norm(pts3D_beforeBA[0]-pts3D_beforeBA[0])-landmark_width)/landmark_width*100<<" %"<<endl;
    cout<<"width_bottom_error_rate: "<<(norm(pts3D_beforeBA[2]-pts3D_beforeBA[3])-landmark_width)/landmark_width*100<<" %"<<endl;
    cout<<"height_left_error_rate: "<<(norm(pts3D_beforeBA[0]-pts3D_beforeBA[2])-landmark_height)/landmark_height*100<<" %"<<endl;
    cout<<"height_right_error_rate: "<<(norm(pts3D_beforeBA[1]-pts3D_beforeBA[3])-landmark_height)/landmark_height*100<<" %"<<endl;



    Optimizer::BundleAdjustment(Frames, Seeds);



    //Reprojection error
    average_error = 0.0;
    n = 0;
    for(int i=0; i<Frames.size(); i++)
    {
        Mat plot_cur = Frames[i]->GetImgUndistorted().clone();

        for(int j=0; j<Seeds.size(); j++)
        {
            Point2f seed_repro = Reprojection(Seeds[j], Frames[i]);
            SignVerticeType seed_type =  Seeds[j]->mType;
            
            for(int k=0; k<Frames[i]->GetTrafficSigns()->mVertices.size(); k++)         //遍歷所有畫面中的角點   
            {
                if(seed_type == Frames[i]->GetTrafficSigns()->mTypes[k])
                {
                    cv::Point2f pt = Frames[i]->GetTrafficSigns()->mVertices[k];
                    float error = norm(seed_repro-pt);
                    average_error += error;
                    n++;

                    cv::circle(plot_cur, seed_repro, 2, Scalar(0,0,255), -1);
                    cv::circle(plot_cur, pt, 2, Scalar(255,0,0), -1);
                }
            }
        }

        imshow("plot_cur", plot_cur);
        waitKey(0); 
    }

    cout<<"average vertices error after BA:  "<<average_error/n<<endl;

    vector<Point3f> pts3D_afterBA;
    for(int i=0; i<Seeds.size(); i++)
    {
        pts3D_afterBA.push_back(Seeds[i]->GetWorldPose());
    }


    //ground truth error
    cout<<"width_up_error_rate: "<<(norm(pts3D_afterBA[0]-pts3D_afterBA[0])-landmark_width)/landmark_width*100<<" %"<<endl;
    cout<<"width_bottom_error_rate: "<<(norm(pts3D_afterBA[2]-pts3D_afterBA[3])-landmark_width)/landmark_width*100<<" %"<<endl;
    cout<<"height_left_error_rate: "<<(norm(pts3D_afterBA[0]-pts3D_afterBA[2])-landmark_height)/landmark_height*100<<" %"<<endl;
    cout<<"height_right_error_rate: "<<(norm(pts3D_afterBA[1]-pts3D_afterBA[3])-landmark_height)/landmark_height*100<<" %"<<endl;


    //VISUALIZE
    std::vector<cv::Point3f> pts3D;
    std::vector<cv::Vec3b> pts_color;
    for(int i=0; i<Frames.size(); i++)
    {
        pts3D.push_back(Converter::toCvPoint3f(Frames[i]->GetCameraCenter()));
        pts_color.push_back(Vec3b(0,255,0));
    }
    for(int i=0; i<pts3D_beforeBA.size(); i++)
    {
        pts3D.push_back(pts3D_beforeBA[i]);
        pts_color.push_back(Vec3b(255,255,255));
    }
    for(int i=0; i<pts3D_afterBA.size(); i++)
    {
        pts3D.push_back(pts3D_afterBA[i]);
        pts_color.push_back(Vec3b(255,255,0));
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr = PCL::generatePointCloudColor(pts3D, pts_color);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = PCL::rgbVis(point_cloud_ptr);
    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

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
    Mat T_w_ref = T_w_cs[121];
 
    //cur
    Mat cur = imread("../data_movingtest/imgs/"+imagenames[131]+".png", 0);    //gray_scale
    Mat T_w_cur = T_w_cs[131];

    //init frames
    TrafficSigns* Signs;
    Frame* frame1 = new Frame(ref, K, DistCoef, Signs);
    frame1->SetPose(T_w_ref.inv());

    Frame* frame2 = new Frame(cur, K, DistCoef, Signs);
    frame2->SetPose(T_w_cur.inv());


    //draw ref, cur
    Mat plot_ref, plot_cur;        
    cvtColor(frame1->GetImgUndistorted(), plot_ref, CV_GRAY2BGR);
    cvtColor(frame2->GetImgUndistorted(), plot_cur, CV_GRAY2BGR);


    //get epipole
    Point2f epipole = GetEpipole(frame1, frame2);
    cv::circle(plot_cur, epipole, 5, Scalar(255,255,0), -1);


    //testing points
    vector<Point2f> pts;
    pts.push_back(Point2f(600,130));
    //pts.push_back(Point2f(866,166));
    pts.push_back(Point2f(693,530));
    pts.push_back(Point2f(681,227));

    for(int i=0; i<pts.size(); i++)
    {
        cv::circle(plot_ref, pts[i], 3, Scalar(255,0,0), -1);
        Point3f line = GetEpipolarLine(frame1, frame2, pts[i]);

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
bool Mapping::UpdateSeed(Seed* seed, Frame* new_frame, const cv::Point2f& new_pt)
{

    //triangulation
    float fx = seed->mframe->fx;
    float fy = seed->mframe->fy;
    float cx = seed->mframe->cx;
    float cy = seed->mframe->cy;

    Mat Tcw1 = seed->mframe->GetPose();
    Mat Tcw2 = new_frame->GetPose();
    Point2f pt1 = seed->mpt;
    Point3f pt1_cam = Point3f((pt1.x-cx)/fx,(pt1.y-cy)/fy,1);
    Point3f pt2_cam = Point3f((new_pt.x-cx)/fx,(new_pt.y-cy)/fy,1);

    Point3f x3Dp;//global
    bool ret = Triangulation(Tcw1, Tcw2, pt1_cam, pt2_cam, x3Dp);

    if(!ret)
        return false;

    //computeTau
    Mat Rcw1 = seed->mframe->GetRotation();
    Mat tcw1 = seed->mframe->GetTranslation();
    Mat x3Dp1_ = Rcw1*Converter::toCvMat(x3Dp) + tcw1;
    Point3f x3Dp1 = Converter::toCvPoint3f(x3Dp1_);     //seed's frame

    Mat T12 = Tcw1 * Tcw2.inv();                                
    float px_noise = 1.0;
    float px_error_angle = atan(px_noise/(2.0*fx))*2.0; // law of chord (sehnensatz)
    
    float tau;
    Depth_Filter::computeTau(T12, x3Dp1, px_error_angle, tau);

    //updateseed
    float z = norm(x3Dp1);
    float tau_inverse = 0.5 * (1.0/fmax(0.0001, z-tau) - 1.0/(z+tau));
    Depth_Filter::updateFilter(1./z, tau_inverse*tau_inverse, seed);
}



// |xp2  - p0 |     |0|
// |yp2  - p1 | X = |0| ===> AX = 0
// |x'p2'- p0'|     |0|
// |y'p2'- p1'|     |0|
bool Mapping::Triangulation(const Mat& Tcw1, const Mat& Tcw2, const Point3f& pt1_cam, const Point3f& pt2_cam, Point3f& x3Dp)
{


    cv::Mat A(4,4, CV_32F);
    A.row(0) = pt1_cam.x*Tcw1.row(2)-Tcw1.row(0);
    A.row(1) = pt1_cam.y*Tcw1.row(2)-Tcw1.row(1);
    A.row(2) = pt2_cam.x*Tcw2.row(2)-Tcw2.row(0);
    A.row(3) = pt2_cam.y*Tcw2.row(2)-Tcw2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    cv::Mat x3D = vt.row(3).t();

    if(x3D.at<float>(3) == 0)
        return false;
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);  

    // check triangulation in front of camera.
    cv::Mat Rcw1, tcw1;
   	Tcw1.rowRange(0,3).colRange(0,3).copyTo(Rcw1);
    Tcw1.rowRange(0,3).col(3).copyTo(tcw1);
    
    cv::Mat Rcw2, tcw2;
   	Tcw2.colRange(0,3).copyTo(Rcw2);
    Tcw2.col(3).copyTo(tcw2);

    cv::Mat x3Dt = x3D.t();

    float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
    if(z1<=0)
        return false;
        
    float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
    if(z2<=0)
        return false;

    x3Dp = Point3f(x3D.at<float>(0), x3D.at<float>(1), x3D.at<float>(2));

}

cv::Mat Mapping::ComputeF21(Frame* frame1, Frame* frame2)
{

	Mat Tcw1 = frame1->GetPose();
	Mat Tcw2 = frame2->GetPose();
	Mat T21 = Tcw2*Tcw1.inv();

    Mat R21, t21;
   	T21.rowRange(0,3).colRange(0,3).copyTo(R21);
    T21.rowRange(0,3).col(3).copyTo(t21);
    
    Mat t21x = ( Mat_<float> ( 3,3 ) <<
                0,                      -t21.at<float> (2),   t21.at<float> (1),
                t21.at<float> (2),     0,                     -t21.at<float> (0),
                -t21.at<float> (1),    t21.at<float> (0),    0);

    Mat E = t21x * R21;

    Mat K1 = frame1->mK;
    Mat K2 = frame2->mK;
    Mat F = K2.inv().t() * E * K1.inv();

    //clean up F
    Eigen::MatrixXf f(3,3);
    f(0,0) = F.at<float>(0,0); f(0,1) = F.at<float>(0,1); f(0,2) = F.at<float>(0,2);
    f(1,0) = F.at<float>(1,0); f(1,1) = F.at<float>(1,1); f(1,2) = F.at<float>(1,2);
    f(2,0) = F.at<float>(2,0); f(2,1) = F.at<float>(2,1); f(2,2) = F.at<float>(2,2);

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(f, Eigen::ComputeFullV | Eigen::ComputeFullU); // ComputeThinU | ComputeThinV
    Eigen::MatrixXf singular_values = svd.singularValues();
    Eigen::MatrixXf left_singular_vectors = svd.matrixU();
    Eigen::MatrixXf right_singular_vectors = svd.matrixV();
    
    Eigen::MatrixXf d(3,3);
    d(0,0) = singular_values(0); d(0,1) = 0;                  d(0,2) = 0;
    d(1,0) = 0;                  d(1,1) = singular_values(1); d(1,2) = 0;
    d(2,0) = 0;                  d(2,1) = 0;                  d(2,2) = 0;

    f = left_singular_vectors*d*right_singular_vectors.transpose();
    F = ( Mat_<float> ( 3,3 ) << f(0,0), f(0,1), f(0,2), f(1,0), f(1,1), f(1,2), f(2,0), f(2,1), f(2,2));

    return F;
}

cv::Point2f Mapping::GetEpipole(Frame* frame1, Frame* frame2)
{

	Mat Rcw2 = frame2->GetRotation();
	Mat tcw2 = frame2->GetTranslation();

	Mat Ow1 = frame1->GetCameraCenter();
	Mat Oc2w1 = Rcw2 * Ow1 + tcw2;

	float fx = frame1->fx;
	float fy = frame1->fy;
	float cx = frame1->cx;
	float cy = frame1->cy;

	return Point2f(Oc2w1.at<float>(0)*fx/Oc2w1.at<float>(2) + cx, Oc2w1.at<float>(1)*fy/Oc2w1.at<float>(2) + cy);
}

cv::Point3f Mapping::GetEpipolarLine(Frame* frame1, Frame* frame2, const cv::Point2f& pt1)
{

    Mat F21 = ComputeF21(frame1, frame2);
	Mat pt1_ = ( Mat_<float> ( 3,1 ) <<  pt1.x, pt1.y, 1 );
	Mat line = F21 * pt1_;
    
    return Point3f(line.at<float>(0,0), line.at<float>(1,0), line.at<float>(2,0));
}

float Mapping::DistPt2Line(const cv::Point2f& pt, const cv::Point3f& line)
{
    return abs(pt.x*line.x + pt.y*line.y + line.z)/sqrt(line.x*line.x+line.y*line.y);   
}


cv::Point2f Mapping::Reprojection(Seed* seed, Frame* frame)
{

    Point3f Pw = seed->GetWorldPose();
    Mat Pc_ = frame->GetRotation()*Converter::toCvMat(Pw) + frame->GetTranslation();
    Point3f Pc = Converter::toCvPoint3f(Pc_);

    float fx = frame->fx;
    float fy = frame->fy;
    float cx = frame->cx;
    float cy = frame->cy;

    return Point2f(Pc.x*fx/Pc.z + cx, Pc.y*fy/Pc.z + cy);
}


void Mapping::GetParameter(cv::Mat& K, cv::Mat& DistCoef, int& width, int& height)
{
    width = 1280;
    height = 720;

    K = cv::Mat(3,3, CV_32F);

    K.at<float>(0,0) = 712.58465576171875000; K.at<float>(0,1) = 0; K.at<float>(0,2) = 613.71289062500000000;
    K.at<float>(1,0) = 0; K.at<float>(1,1) = 713.57849121093750000; K.at<float>(1,2) = 386.50469970703125000;
    K.at<float>(2,0) = 0; K.at<float>(2,1) = 0; K.at<float>(2,2) = 1;

    DistCoef= ( Mat_<float> ( 1,4 ) << -0.29970550537109375, 0.07715606689453125, -0.00035858154296875, 0.00141906738281250);
}



bool Mapping::findCorresponding( const Mat& img1, const Mat& img2, vector<Point2f>& pts1, vector<Point2f>& pts2 )
{

    Ptr<SURF> detector = SURF::create();
    detector->setHessianThreshold(400);
    std::vector<KeyPoint> kps1, kps2;
    Mat descriptors1, descriptors2;
    detector->detectAndCompute( img1, Mat(), kps1, descriptors1 );
    detector->detectAndCompute( img2, Mat(), kps2, descriptors2 );

    cout<<"分别找到了"<<kps1.size()<<"和"<<kps2.size()<<"个特征点"<<endl;
    

    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors1, descriptors2, matches );

    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < descriptors1.rows; i++ ){ 
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors1.rows; i++ ){
        if( matches[i].distance <= max(2*min_dist, 0.02) ){ 
            good_matches.push_back( matches[i]); 
        }
    }

    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches( img1, kps1, img2, kps2,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    //-- Show detected matches
    imshow( "Good Matches", img_matches );

    if (good_matches.size() <= 18) //匹配点太少
        return false;

    //-- Localize
    for( size_t i = 0; i < good_matches.size(); i++ ){
        //-- Get the keypoints from the good matches
        Point2f pt1 = kps1[ good_matches[i].queryIdx ].pt;
        Point2f pt2 = kps2[ good_matches[i].trainIdx ].pt;
        pts1.push_back( pt1 );
        pts2.push_back( pt2 );
    }

    return true;
}
