#include "mapping.h"

#include "converter.h"
#include "depth_filter.h"
#include "optimizer.h"
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
    this->map = new Map();
}

void Mapping::CreateMap()
{
    vector<Frame*> Frames_ = CreateFrames();
    vector<Frame*> Frames;
    for(int i=Frames_.size()-1; i>=0; i--)  //Reverse frame
        Frames.push_back(Frames_[i]);

    //Create Sign
    //this->map->AddSign(CreateSign(Frames[0], Frames[0]->GetSignLabels()[0]));    //Init sign0(0-37), sign1(25-56)
    this->map->AddSign(CreateSign(Frames[25], Frames[25]->GetSignLabels()[0]));  //Init sign1(25-56)

    //Sign* sign0 = this->map->GetAllSigns()[0];
    Sign* sign1 = this->map->GetAllSigns()[0];


    PlotSign(sign1);
    PlotSignObs(sign1);


    //Update Sign0 
    // for(int i=1; i<38; i++)
    // {
    //     int index = 0;
    //     if (Frames[i]->GetSignLabels().size()==2)
    //         index = 1;
        
    //     UpdateSign(sign0, Frames[i], Frames[i]->GetSignLabels()[index]);
    // }

    //Update Sign1
    for(int i=26; i<57; i++)
    {      
        UpdateSign(sign1, Frames[i], Frames[i]->GetSignLabels()[0]);
    }

    cout<<"avg_px_error_sign1: "<<AvgReprojectionError(sign1)<<endl; 
    GroundTruthError1(sign1);

    Optimizer::LocalBundleAdjustment(sign1,10);

    cout<<"avg_px_error_sign1: "<<AvgReprojectionError(sign1)<<endl; 
    GroundTruthError1(sign1);

}



void Mapping::CreateMap2()
{

    vector<Frame*> Frames_ = CreateFrames();
    vector<Frame*> Frames;
    for(int i=Frames_.size()-1; i>=0; i--)  //Reverse frame
        Frames.push_back(Frames_[i]);

    Sign* sign = CreateSign(Frames[25], Frames[25]->GetSignLabels()[0]);
    this->map->AddSign(sign);
    
    std::vector<MapPoint*> MapPoints = CreateMappoints(Frames[25]);
    for(int i=0; i<MapPoints.size(); i++)
        this->map->AddMappoint(MapPoints[i]);


    //PlotSign(sign);
    //PlotMappoints(MapPoints);











}



std::vector<Frame*> Mapping::CreateFrames()
{

    Mat K = cv::Mat(3,3, CV_32F);
    K.at<float>(0,0) = 712.58465576171875000; K.at<float>(0,1) = 0; K.at<float>(0,2) = 613.71289062500000000;
    K.at<float>(1,0) = 0; K.at<float>(1,1) = 713.57849121093750000; K.at<float>(1,2) = 386.50469970703125000;
    K.at<float>(2,0) = 0; K.at<float>(2,1) = 0; K.at<float>(2,2) = 1;

    vector<string> Imagenames; 
    vector<vector<Point2f>> Verticess;
    Utils::ReadVertices("../imgs/line1.csv", Imagenames, Verticess);

    vector<Mat> Twcs;
    Twcs = Utils::ReadCameraPoses("../line1-vins_estimator-camera_pose.csv", Imagenames);

    int n_data = Imagenames.size();
    
    vector<Mat> Imgs;
    for(int i=0; i<n_data; i++)
    {
        Imgs.push_back(imread("../imgs/imgs_line1_undistort/"+Imagenames[i], 1));
    }

    //create frame;
    Ptr<SURF> detector = SURF::create();
    detector->setHessianThreshold(400);

    std::vector<Frame*> Frames_;
    for(int i=0; i<n_data; i++)
    {
        int n_label;

        if(Verticess[i].size() == 4)
            n_label = 1;
        else if (Verticess[i].size() == 8)
            n_label = 2;

        vector<Frame::SignLabel*> Labels;

        for(int j=0; j<n_label; j++)
        {
            Frame::SignLabel* label = new Frame::SignLabel();
            label->Vertices.push_back(Verticess[i][j*4]);
            label->Vertices.push_back(Verticess[i][j*4+1]);
            label->Vertices.push_back(Verticess[i][j*4+2]);
            label->Vertices.push_back(Verticess[i][j*4+3]);

            Labels.push_back(label);
        }

        Frame* frame = new Frame(Imgs[i], K, Labels, detector);
        frame->SetPose(Twcs[i].inv());
        Frames_.push_back(frame);
    }

    return Frames_;
}



Sign* Mapping::CreateSign(Frame* frame, Frame::SignLabel* label)
{
    std::vector<Seed*> Seeds;
    Seeds.push_back(new Seed(frame, label->Vertices[0], 1.0, 0.1));
    Seeds.push_back(new Seed(frame, label->Vertices[1], 1.0, 0.1));
    Seeds.push_back(new Seed(frame, label->Vertices[2], 1.0, 0.1));
    Seeds.push_back(new Seed(frame, label->Vertices[3], 1.0, 0.1));
    Sign* sign = new Sign(Seeds, frame);
    sign->AddObservation(frame, label);
    return sign;
}

std::vector<MapPoint*> Mapping::CreateMappoints(Frame* frame)
{
    std::vector<MapPoint*> MapPoints;

    std::vector<cv::KeyPoint> KeyPoints = frame->GetKepPoints();

    for(int i=0; i<KeyPoints.size(); i++)
    {
        MapPoint* mappoint = new MapPoint(new Seed(frame, KeyPoints[i].pt, 1.0, 0.1), frame);
        mappoint->AddObservation(frame, KeyPoints[i].pt);
        MapPoints.push_back(mappoint);
    }

    return MapPoints;
}







bool Mapping::UpdateSign(Sign* sign, Frame* frame, Frame::SignLabel* label)
{

    Point3f Ow_sign = Converter::toCvPoint3f(sign->GetFirstFrame()->GetCameraCenter());
    Point3f Ow_cur = Converter::toCvPoint3f(frame->GetCameraCenter());

    float dist_Ows = norm(Ow_cur-Ow_sign);

    //ensure long enough
    if(dist_Ows<0.3)
        return false;

    float fx = frame->fx;
    float fy = frame->fy;
    float cx = frame->cx;
    float cy = frame->cy;

    Mat Tcw1 = sign->GetFirstFrame()->GetPose();
    Mat Tcw2 = frame->GetPose();

    //Triangulation //global pose
    vector<Point3f> pts3D;
    for(int i=0; i<4; i++)
    {
        Point3f pt1_cam = Utils::Px2Cam(sign->GetAllSeeds()[i]->mpt, fx, fy, cx, cy);
        Point3f pt2_cam = Utils::Px2Cam(label->Vertices[i], fx, fy, cx, cy);
       
        Point3f pt3D;
        bool ret = Utils::Triangulation(Tcw1, Tcw2, pt1_cam, pt2_cam, pt3D);  

        if(!ret)
            return false;

        pts3D.push_back(pt3D);
    }

    if(ReprojectionError(pts3D, sign)>4.0)
        return false;

    if(ReprojectionError(pts3D, frame, label)>4.0)
        return false;

    //Depth filter
    Mat T12 = Tcw1 * Tcw2.inv(); 
    float px_noise = 1.0;
    float px_error_angle = atan(px_noise/(2.0*fx))*2.0; // law of chord (sehnensatz)
    
    for(int i=0; i<4; i++)
    {
        Mat Rcw1 = sign->GetFirstFrame()->GetRotation();
        Mat tcw1 = sign->GetFirstFrame()->GetTranslation();
        Mat pt3D1_ = Rcw1*Converter::toCvMat(pts3D[i]) + tcw1;
        Point3f pt3D1 = Converter::toCvPoint3f(pt3D1_);     //seed's frame

        float tau;       
        Depth_Filter::computeTau(T12, pt3D1, px_error_angle, tau);    //compute tau

        float z = norm(pt3D1);
        float tau_inverse = 0.5 * (1.0/fmax(0.0001, z-tau) - 1.0/(z+tau));
        Depth_Filter::updateFilter(1./z, tau_inverse*tau_inverse, sign->GetAllSeeds()[i]);

        //cout<< sign->GetAllSeeds()[i]->mu <<"    "<< sign->GetAllSeeds()[i]->sigma2<<endl;  
    }

    //update
    sign->AddObservation(frame, label);
}



float Mapping::AvgReprojectionError(Sign* sign)
{

    vector<Point3f> pts3D;
    for(int i=0; i<4; i++)
        pts3D.push_back(sign->GetAllSeeds()[i]->GetWorldPose());

    float avg_px_error_sign = 0.0;
    
    for(int i=0; i<sign->nObs; i++)
    {
        Frame* frame = sign->GetObservationFrames()[i];
        Frame::SignLabel* label = sign->GetObservationLabels()[i];
        avg_px_error_sign += ReprojectionError(pts3D, frame, label);
        PlotReprojections(pts3D, frame, label);
    }

    return avg_px_error_sign /= sign->nObs;
}



float Mapping::ReprojectionError(std::vector<cv::Point3f> pts3D, Sign* sign)
{
    float avg_error = 0.0;

    for(int i=0; i<pts3D.size(); i++)
    {
        Point3f Pw = pts3D[i];
        Mat Pc_ = sign->GetFirstFrame()->GetRotation()*Converter::toCvMat(Pw) + sign->GetFirstFrame()->GetTranslation();
        Point3f Pc = Converter::toCvPoint3f(Pc_);
        
        //to pixel frame
        float fx = sign->GetFirstFrame()->fx;
        float fy = sign->GetFirstFrame()->fy;
        float cx = sign->GetFirstFrame()->cx;
        float cy = sign->GetFirstFrame()->cy;

        Point2f px = Utils::Cam2Px(Pc, fx, fy, cx, cy);
        avg_error+=norm(px-sign->GetAllSeeds()[i]->mpt);
    }

    return avg_error/=pts3D.size();
}



float Mapping::ReprojectionError(std::vector<cv::Point3f> pts3D, Frame* frame, Frame::SignLabel* label)
{
    float avg_error = 0.0;

    for(int i=0; i<pts3D.size(); i++)
    {
        Point3f Pw = pts3D[i];
        Mat Pc_ = frame->GetRotation()*Converter::toCvMat(Pw) + frame->GetTranslation();
        Point3f Pc = Converter::toCvPoint3f(Pc_);
        
        //to pixel frame
        float fx = frame->fx;
        float fy = frame->fy;
        float cx = frame->cx;
        float cy = frame->cy;

        Point2f px = Utils::Cam2Px(Pc, fx, fy, cx, cy);
        avg_error+=norm(px-label->Vertices[i]);
    }

    return avg_error/=pts3D.size();
}



std::vector<cv::Point2f> Mapping::GetEopipolarSegment(Seed* seed, Frame* frame)
{

    Point3f pose_min = seed->GetWorldPoseMin();
    Point3f pose_max = seed->GetWorldPoseMax();
    
    //to camera frame
    Mat Rcw = frame->GetRotation();
    Mat tcw = frame->GetTranslation();

    Mat pose_min_cam_ = Rcw*Converter::toCvMat(pose_min) + tcw;
    Mat pose_max_cam_ = Rcw*Converter::toCvMat(pose_max) + tcw;

    Point3f pose_min_cam = Converter::toCvPoint3f(pose_min_cam_);
    Point3f pose_max_cam = Converter::toCvPoint3f(pose_max_cam_);
    
    //to pixel frame
    float fx = frame->fx;
    float fy = frame->fy;
    float cx = frame->cx;
    float cy = frame->cy;

    Point2f pose_min_px = Utils::Cam2Px(pose_min_cam, fx, fy, cx, cy);
    Point2f pose_max_px = Utils::Cam2Px(pose_max_cam, fx, fy, cx, cy);
    
    vector<Point2f> Segment;
    Segment.push_back(pose_min_px);
    Segment.push_back(pose_max_px);

    return Segment;
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
    Mat F21 = Utils::ComputeF21(frame1->GetPose(), frame2->GetPose(), frame1->mK);
    Mat pt1_ = ( Mat_<float> ( 3,1 ) <<  pt1.x, pt1.y, 1 );
    Mat line = F21 * pt1_;
    return Point3f(line.at<float>(0,0), line.at<float>(1,0), line.at<float>(2,0));
}

void Mapping::GroundTruthError0(Sign* sign0)
{

    float width_gt = 0.31;
    float height_gt = 0.174;
    float sign0_LT_gt = 3.95;

    float sign0_width_err_rate = abs((sign0->GetAvgWidth() - width_gt))/width_gt*100;
    float sign0_height_err_rate = abs((sign0->GetAvgHeight() - height_gt))/height_gt*100;

    float sign0_LT_err_rate = abs(( norm(sign0->GetAllSeeds()[0]->GetWorldPose()) - sign0_LT_gt )) / sign0_LT_gt*100;

    cout<<"sign0_width_err_rate: "<<sign0_width_err_rate<<" %"<<endl;
    cout<<"sign0_height_err_rate: "<<sign0_height_err_rate<<" %"<<endl;
    cout<<"sign0_LT_err_rate: "<<sign0_LT_err_rate<<" %"<<endl;
   
}


void Mapping::GroundTruthError1(Sign* sign1)
{

    float width_gt = 0.31;
    float height_gt = 0.174;
    float sign1_LT_gt = 3.0;

    float sign1_width_err_rate = abs((sign1->GetAvgWidth() - width_gt))/width_gt*100;
    float sign1_height_err_rate = abs((sign1->GetAvgHeight() - height_gt))/height_gt*100;

    float sign1_LT_err_rate = abs(( norm(sign1->GetAllSeeds()[0]->GetWorldPose()) - sign1_LT_gt )) / sign1_LT_gt*100;

    cout<<"sign1_width_err_rate: "<<sign1_width_err_rate<<" %"<<endl;
    cout<<"sign1_height_err_rate: "<<sign1_height_err_rate<<" %"<<endl;
    cout<<"sign1_LT_err_rate: "<<sign1_LT_err_rate<<" %"<<endl;

}






void Mapping::PlotLabel(Frame* frame, Frame::SignLabel* label)
{
    Mat img = frame->GetImg();
    cv::circle(img, label->Vertices[0], 2, Scalar(255,0,0), -1);
    cv::circle(img, label->Vertices[1], 2, Scalar(255,0,0), -1);
    cv::circle(img, label->Vertices[2], 2, Scalar(255,0,0), -1);
    cv::circle(img, label->Vertices[3], 2, Scalar(255,0,0), -1);
    imshow("img", img);
    waitKey(0);
}

void Mapping::PlotSign(Sign* sign)
{
    Mat img =  sign->GetFirstFrame()->GetImg();
    cv::circle(img, sign->GetAllSeeds()[0]->mpt, 2, Scalar(255,0,0), -1);
    cv::circle(img, sign->GetAllSeeds()[1]->mpt, 2, Scalar(255,0,0), -1);
    cv::circle(img, sign->GetAllSeeds()[2]->mpt, 2, Scalar(255,0,0), -1);
    cv::circle(img, sign->GetAllSeeds()[3]->mpt, 2, Scalar(255,0,0), -1);
    imshow("img", img);
    waitKey(0);
}

void Mapping::PlotSignObs(Sign* sign)
{
    for(int i=0; i<sign->nObs; i++)
    {
        Frame* frame = sign->GetObservationFrames()[i];
        Frame::SignLabel* label = sign->GetObservationLabels()[i];
        PlotLabel(frame, label);
    }
}

void Mapping::PlotMappoints(std::vector<MapPoint*> MapPoints)
{
    Mat img =  MapPoints[0]->GetFirstFrame()->GetImg();
    for(int i=0; i<MapPoints.size(); i++)
    {
        cv::circle(img, MapPoints[i]->GetSeed()->mpt, 2, Scalar(255,0,0), -1);
    }
    imshow("img", img);
    waitKey(0);
}










void Mapping::PlotReprojections(vector<Point3f> pts3D, Sign* sign)
{
    Mat img = sign->GetFirstFrame()->GetImg();

    for(int i=0; i<pts3D.size(); i++)
    {
        Point3f Pw = pts3D[i];
        Mat Pc_ = sign->GetFirstFrame()->GetRotation()*Converter::toCvMat(Pw) + sign->GetFirstFrame()->GetTranslation();
        Point3f Pc = Converter::toCvPoint3f(Pc_);
        
        //to pixel frame
        float fx = sign->GetFirstFrame()->fx;
        float fy = sign->GetFirstFrame()->fy;
        float cx = sign->GetFirstFrame()->cx;
        float cy = sign->GetFirstFrame()->cy;

        Point2f px = Utils::Cam2Px(Pc, fx, fy, cx, cy);

        cv::circle(img, px, 2, Scalar(0,0,255), -1);
        cv::circle(img, sign->GetAllSeeds()[i]->mpt, 2, Scalar(255,0,0), -1);
    }
    imshow("img", img);
    waitKey(0);
}

void Mapping::PlotReprojections(std::vector<cv::Point3f> pts3D, Frame* frame, Frame::SignLabel* label)
{
    Mat img = frame->GetImg();

    for(int i=0; i<pts3D.size(); i++)
    {
        Point3f Pw = pts3D[i];
        Mat Pc_ = frame->GetRotation()*Converter::toCvMat(Pw) + frame->GetTranslation();
        Point3f Pc = Converter::toCvPoint3f(Pc_);
        
        //to pixel frame
        float fx = frame->fx;
        float fy = frame->fy;
        float cx = frame->cx;
        float cy = frame->cy;

        Point2f px = Utils::Cam2Px(Pc, fx, fy, cx, cy);

        cv::circle(img, px, 2, Scalar(0,0,255), -1);
        cv::circle(img, label->Vertices[i], 2, Scalar(255,0,0), -1);
    }
    imshow("img", img);
    waitKey(0);
}