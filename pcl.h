#pragma once
#ifndef PCL_H
#define PCL_H

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <opencv2/core/core.hpp>

class PCL
{
public:
	PCL();
	~PCL();
	pcl::PointCloud<pcl::PointXYZ>::Ptr generatePointCloud(std::vector<cv::Point3d> pts3D);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloudColor(std::vector<cv::Point3d> pts3D, std::vector<cv::Vec3b> pts_color);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
};
#endif //PCL_H