#pragma once
#ifndef MAPPING_H
#define MAPPING_H

#include "frame.h"
#include "seed.h"
#include <opencv2/opencv.hpp>

class Mapping
{
public:
	Mapping();

	void CreateMapPoints();
	void MovingObjectTest();
	void TestBA();


private:
	//update depth filter
	bool UpdateSeed(Seed* seed, Frame* new_frame, const cv::Point2f& new_pt);
	//compute 3D point based on world frame
    bool Triangulation(const cv::Mat& Tcw1, const cv::Mat& Tcw2, const cv::Point3f& pt1_cam, const cv::Point3f& pt2_cam, cv::Point3f& x3Dp);
    cv::Mat ComputeF21(Frame* frame1, Frame* frame2);
    cv::Point2f GetEpipole(Frame* frame1, Frame* frame2);
    cv::Point3f GetEpipolarLine(Frame* frame1, Frame* frame2, const cv::Point2f& pt1);
    float DistPt2Line(const cv::Point2f& pt, const cv::Point3f& line);
    cv::Point2f Reprojection(Seed* seed, Frame* frame);

private:
	void GetParameter(cv::Mat& K, cv::Mat& DistCoef, int& width, int& height);
	bool findCorresponding( const cv::Mat& img1, const cv::Mat& img2, std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2 );
};
#endif //MAPPING_H