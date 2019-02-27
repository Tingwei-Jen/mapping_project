#pragma once
#ifndef TRAFFICSIGN_H
#define TRAFFICSIGN_H

#include <opencv2/opencv.hpp>

enum SignVerticeType
{
	TriangleUp,
	UpLeft,
	UpRight,
	DownLeft,
	DownRight
};

class TrafficSigns
{
public:
	TrafficSigns(){}
	std::vector<cv::Point2f> mVertices;
	std::vector<SignVerticeType> mTypes;

};

#endif //TRAFFICSIGN_H