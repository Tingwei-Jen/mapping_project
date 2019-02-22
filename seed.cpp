#include "seed.h"
#include "converter.h"

int Seed::counter = 0;

Seed::Seed(Frame* frame, cv::Point2d& pt, double& depth_mean, double& depth_min)
:mId(counter++), mpt(pt), a(10), b(10), mu(1.0/depth_mean), z_range(1.0/depth_min)
{

	mframe = frame;
    sigma2 = z_range*z_range/36;
    
    cv::Point3d pt_cam = cv::Point3d((pt.x-mframe->cx)/mframe->fx,(pt.y-mframe->cy)/mframe->fy,1);
    mf = pt_cam/norm(pt_cam);
}

cv::Point3d Seed::GetWorldPose()
{
	cv::Point3d p = mf * (1/mu);
	cv::Mat p_ = Converter::toCvMat(p);
	cv::Mat Rwc = mframe->GetRotationInverse();
	cv::Mat twc = mframe->GetCameraCenter();

    cv::Mat pw_ = Rwc * p_ + twc;
    cv::Point3d pw = Converter::toCvPoint3d(pw_);

    return pw; 
}