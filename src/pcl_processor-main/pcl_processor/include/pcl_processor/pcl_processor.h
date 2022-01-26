#ifndef ROS_PCL_PROCESSOR_H
#define ROS_PLC_PROCESSOR_H

#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/common.h>

#include <geometry_msgs/Vector3.h>

class PclProcessor
{
  protected:
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
	geometry_msgs::Vector3 object_position;
  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;// (new pcl::PointCloud<pcl::PointXYZ>);


  public:

  PclProcessor();
  ~PclProcessor();

  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud();
  geometry_msgs::Vector3 getObjectPosition();

  void process();
};

#endif
