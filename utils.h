#pragma once
#ifndef UTILS_H
#define UTILS_H

#include <opencv2/core/core.hpp>

class Utils
{
public:
    static void readCsv(std::string path, int n_data, std::vector<cv::Mat>& T_w_cs, std::vector<std::string>& imagenames);
    static void readMonitorVertex(std::vector<std::vector<cv::Point2d>>& vertexss);    //index 25---64
    static void readLandMarkVertex(std::vector<std::vector<cv::Point2d>>& vertexss);
    static cv::Mat quaternion2RotM(double x, double y, double z, double w);
};
#endif //MAPPING_H