#ifndef ROS_PCL_PROCESSOR_NODE_H
#define ROS_PCL_PROCESSOR_NODE_H

#include "ros/ros.h"
#include "pcl_processor_msgs/CalculateObjectposeFromPointcloud.h"
#include "pcl_processor_msgs/CalculateObjectposeFromPointcloudRequest.h"
#include "pcl_processor_msgs/CalculateObjectposeFromPointcloudResponse.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
//#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//Image message
#include <sensor_msgs/Image.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_processor/pcl_processor.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


/** \brief Simple Pcl Processor
 *
 * Simple Pcl Processor with PCL calls
 *
 */
class PclProcessorRos
{
    protected:
        //ros node handle
        ros::NodeHandle nh_;
        PclProcessor pcl_processor;
        ros::Publisher filtered_pointcloud_pub;
        ros::Publisher capture_pointcloud_pub;
        ros::ServiceServer calculate_objectpose_service;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl_cloud;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr capture_pcl_cloud;

    protected:

    public:
        /** \brief Constructor
        *
        * Constructor
        *
        */
        PclProcessorRos();

        /** \brief Destructor
        *
        * Destructor
        *
        */
        ~PclProcessorRos();


        /** \brief Process input Pcl
        *
        * Process input Pcl
        *
        **/
        //void process();

        /** \brief Publish output Pcl
        *
        * Publish output Pcl
        *
        */
        //void publish();


        bool CalculateObjectpose(pcl_processor_msgs::CalculateObjectposeFromPointcloud::Request  &request,      pcl_processor_msgs::CalculateObjectposeFromPointcloud::Response &response);
};
#endif
