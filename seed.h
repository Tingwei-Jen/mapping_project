#pragma once
#ifndef SEED_H
#define SEED_H

#include "frame.h"
#include <opencv2/opencv.hpp>

class Seed
{
public:
	Seed(Frame* frame, cv::Point2d& pt, double& depth_mean, double& depth_min);
    cv::Point3d GetWorldPose();

public:
	static int counter;
    Frame* mframe;                 //!< the frame for which the seed was created.
    int mId;                       //!< Seed ID
    cv::Point2d mpt;               //!< feature's pixel location.
    cv::Point3d mf;                //!< unit vector of feature in camera frame
    double a;                      //!< a of Beta distribution: When high, probability of inlier is large.
    double b;                      //!< b of Beta distribution: When high, probability of outlier is large.
    double mu;                     //!< Mean of normal distribution.
    double z_range;                //!< Max range of the possible depth.
    double sigma2;                 //!< Variance of normal distribution.

};

#endif //SEED_H