#include "pcl.h"
#include <iostream>

PCL::PCL()
{
	std::cout<<"Construct PCL"<<std::endl;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr PCL::generatePointCloud(std::vector<cv::Point3d> pts3D)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

	for(int i=0; i<pts3D.size(); i++)
	{
		pcl::PointXYZ basic_point;
		basic_point.x = pts3D[i].x;
		basic_point.y = pts3D[i].y;
		basic_point.z = pts3D[i].z;
		basic_cloud_ptr->points.push_back(basic_point);
	}

  	basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  	basic_cloud_ptr->height = 1;

	return basic_cloud_ptr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCL::generatePointCloudColor(std::vector<cv::Point3d> pts3D, std::vector<cv::Vec3b> pts_color)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

	for(int i=0; i<pts3D.size(); i++)
	{
		uchar blue = pts_color[i].val[0];
		uchar green = pts_color[i].val[1];
		uchar red = pts_color[i].val[2];

		pcl::PointXYZRGB point;
		point.x = pts3D[i].x;
		point.y = pts3D[i].y;
		point.z = pts3D[i].z;
      	point.r = red;
      	point.g = green;
      	point.b = blue;

      	point_cloud_ptr->points.push_back (point);
	}

  	point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  	point_cloud_ptr->height = 1;

	return point_cloud_ptr;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> PCL::simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  viewer->addCoordinateSystem (0.2);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> PCL::rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
  viewer->addCoordinateSystem (0.2);
  viewer->initCameraParameters ();
  return (viewer);
}