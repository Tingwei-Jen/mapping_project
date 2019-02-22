#pragma once
#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp>

class Frame
{
public:
	Frame(const cv::Mat& img, cv::Mat &K, cv::Mat &DistCoef);
	void SetPose(const cv::Mat& Tcw);
    
    cv::Mat GetImg(){ return mImg.clone(); }
	cv::Mat GetPose(){ return mTcw.clone(); }
    cv::Mat GetCameraCenter(){ return mOw.clone(); }
    cv::Mat GetRotation(){ return mRcw.clone(); }
    cv::Mat GetTranslation(){ return mtcw.clone(); }
    cv::Mat GetRotationInverse(){ return mRwc.clone(); }
    std::vector<cv::Point2d> GetVertices(){ return mVertices;}

public:
    static int frame_counter;
    static double fx;
    static double fy;
    static double cx;
    static double cy;
    static double invfx;
    static double invfy;
    int mId;
    cv::Mat mK;
    cv::Mat mDistCoef;

    //the vertices in this frame
    std::vector<cv::Point2d> mVertices;

private:
	cv::Mat mImg; 
	cv::Mat mTcw;  ///< 相机姿态 世界坐标系到相机坐标坐标系的变换矩阵
    cv::Mat mRcw;  ///< Rotation from world to camera
    cv::Mat mRwc;  ///< Rotation from camera to world
    cv::Mat mtcw;  ///< Translation from world to camera   
    cv::Mat mOw;   ///< mtwc,Translation from camera to world
};
#endif //FRAME_H
