#pragma once
#ifndef UTILS_H
#define UTILS_H

#include <opencv2/core/core.hpp>

class Utils
{
public:
    Utils();
    cv::Mat quaternion2RotM(double x, double y, double z, double w);
    void readCsv(std::string path, int n_data, std::vector<cv::Mat>& T_w_cs, std::vector<std::string>& imagenames);
};
#endif //MAPPING_H