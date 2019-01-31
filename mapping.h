#pragma once
#ifndef MAPPING_H
#define MAPPING_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;

class Mapping
{
public:
    Mapping(Mat K, int width, int height, int border);
    ~Mapping();

    //compute distance between camera centers
    void computeDistOfCenters(const Mat& T_w_ref, const Mat& T_w_cur, double& dist);
    //compute epipole in current image
    void computeEpipole(const Mat& T_w_ref, const Mat& T_w_cur, Point2d& epipole);
    //compute epipolar line by F
    void computeEpipolarLine(const Point2d& pt_ref, const Mat& T_w_ref, const Mat& T_w_cur, Point3d& line_cur);
    //compute distant between point and line
    void distPoint2Line(const Point2d& pt_cur, const Point3d& line_cur, double& dist);
    //compute line segment (possible 3D points location) in current frame.
    void epipolarSegment(const Mat& T_w_ref, const Mat& T_w_cur, const Point2d& pt_ref, const double& depth_mu, const double& depth_std, std::vector<Point2d>& pts_cur_temp);
    //search best current match points according similarity along epipolar line
    bool epipolarSearch(Mat ref, Mat cur, Point2d pt_ref, std::vector<Point2d> pts_cur_temp, int window_size, Point2d& best_pt_cur);
    //update depth_mu and depth_cov
    bool updateDepthFilter(Mat T_w_ref, Mat T_w_cur, Point2d pt_ref, Point2d pt_cur, double& depth_mu, double& depth_cov);
    //compute reprojection
    void computeReprojection(Mat T_w_ref, Mat T_w_cur, Point2d pt_ref, double depth, Point2d& pt_cur_repro);

    //find vertical line
    bool verticalLine(const Point3d& line, const Point2d& point_through, Point3d& verticle_line);



private:
    Point3d px2cam (Point2d px);
    Point2d cam2px (Point3d p_cam);
    bool inside(Point2d px);
    void computeF(Mat T_w_ref, Mat T_w_cur, Mat& F);
    double NCC(Mat ref, Mat cur, Point pt_ref, Point2d pt_cur, int window_size);

private:
    Mat m_K; 
    double m_fx;
    double m_fy;
    double m_cx;
    double m_cy;
    int m_width;
    int m_height;
    int m_border;

};
#endif //MAPPING_H