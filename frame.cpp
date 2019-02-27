#include "frame.h"

int Frame::frame_counter = 0;
float Frame::fx, Frame::fy, Frame::cx, Frame::cy, Frame::invfx, Frame::invfy;

Frame::Frame(const cv::Mat& img, cv::Mat &K, cv::Mat &DistCoef, TrafficSigns* Signs)
:mImg(img.clone()), mK(K.clone()), mDistCoef(DistCoef.clone())
{
	
	mId = frame_counter++;

    fx = K.at<float>(0,0);
    fy = K.at<float>(1,1);
    cx = K.at<float>(0,2);
    cy = K.at<float>(1,2);
    invfx = 1.0/fx;
    invfy = 1.0/fy;

    undistort(img, mImgUndistorted, K, DistCoef);
    mSigns = Signs;
}

void Frame::SetPose(const cv::Mat& Tcw)
{

	mTcw = Tcw.clone();
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;

}