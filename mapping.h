#pragma once
#ifndef MAPPING_H
#define MAPPING_H

#include "map.h"
#include <opencv2/opencv.hpp>


class Mapping
{
public:
	Mapping();
	void CreateMap();
	void CreateMap2();   //with other mappoints

	Map* GetMap(){ return map; }

private:
	//create frames
	std::vector<Frame*> CreateFrames();
	//create sign (seed)
	Sign* CreateSign(Frame* frame, Frame::SignLabel* label);
	//use depth filter to update the depth of the seed in the sign.
	bool UpdateSign(Sign* sign, Frame* frame, Frame::SignLabel* label);
	//return reprojection error of all sign observations.
	float AvgReprojectionError(Sign* sign);

private:
	std::vector<MapPoint*> CreateMappoints(Frame* frame);

private:
	float ReprojectionError(std::vector<cv::Point3f> pts3D, Sign* sign);
	float ReprojectionError(std::vector<cv::Point3f> pts3D, Frame* frame, Frame::SignLabel* label);

	std::vector<cv::Point2f> GetEopipolarSegment(Seed* seed, Frame* frame);
    cv::Point2f GetEpipole(Frame* frame1, Frame* frame2);
    cv::Point3f GetEpipolarLine(Frame* frame1, Frame* frame2, const cv::Point2f& pt1);


private:
	void GroundTruthError0(Sign* sign0);
	void GroundTruthError1(Sign* sign1);
	
	void PlotLabel(Frame* frame, Frame::SignLabel* label);	
	void PlotSign(Sign* sign);
	void PlotSignObs(Sign* sign);
	void PlotMappoints(std::vector<MapPoint*> MapPoints);

	void PlotReprojections(std::vector<cv::Point3f> pts3D, Sign* sign);
	void PlotReprojections(std::vector<cv::Point3f> pts3D, Frame* frame, Frame::SignLabel* label);

private:
	Map* map;


};
#endif //MAPPING_H