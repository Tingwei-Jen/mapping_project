#pragma once
#ifndef MAPPING_H
#define MAPPING_H

#include "frame.h"
#include <opencv2/opencv.hpp>

class Mapping
{
public:
	Mapping();

	void CreateMapPoints();
	void MovingObjectTest();



private:
	//compute 3D point based on world frame
    bool Triangulation(const cv::Mat& Tcw1, const cv::Mat& Tcw2, const cv::Point3d& pt_cam1, const cv::Point3d& pt_cam2, cv::Point3d& x3Dp);
    cv::Mat ComputeF21(Frame* frame1, Frame* frame2);
    cv::Point2d GetEpipole(Frame* frame1, Frame* frame2);
    cv::Point3d GetEpipolarLine(Frame* frame1, Frame* frame2, const cv::Point2d& pt1);

private:
	void GetParameter(cv::Mat& K, cv::Mat& DistCoef, int& width, int& height);
	void GetData();

};
#endif //MAPPING_H