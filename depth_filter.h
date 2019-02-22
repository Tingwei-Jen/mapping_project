#pragma once
#ifndef DEPTH_FILTER_H
#define DEPTH_FILTER_H

#include "seed.h"
#include <opencv2/opencv.hpp>

class Depth_Filter
{
public:
    //compute observed depth std
    static void computeTau(const cv::Mat& T_ref_cur, const cv::Point3d& P, const double& px_error_angle, double& tau);
    //gaussian&beta distribution fusion,  (mu, sigma2) combine with (x, tau2)(observation)
    static void updateFilter(const double& x, const double& tau2, Seed* seed);   
};
#endif //DEPTH_FILTER_H