#pragma once
#ifndef UTILS_H
#define UTILS_H

#include <opencv2/core/core.hpp>

using namespace cv;

class Utils
{
public:
    Utils();
    ~Utils();
    cv::Mat quaternion2RotM(double x, double y, double z, double w);
    void readCsv(std::string path, int n_data, std::vector<cv::Mat>& T_w_cs, std::vector<std::string>& imagenames);
    void readMonitorVertex(std::vector<std::vector<Point2d>>& vertexss);    //index 25---64
    void readLandMarkVertex(std::vector<std::vector<Point2d>>& vertexss);
};
#endif //MAPPING_H