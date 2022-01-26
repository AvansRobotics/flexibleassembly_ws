#include "pcl_processor/pcl_processor_node.h" 
#include <cv_bridge/cv_bridge.h> 
#include <sensor_msgs/image_encodings.h> 
#include <sensor_msgs/Image.h> 
#include <image_transport/image_transport.h> 
#include <opencv2/highgui.hpp> 
#include <opencv2/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 

 
 
void imageCallback(const sensor_msgs::ImageConstPtr& msg){ 

try{ 
//cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image); 
//cv::imwrite("test.jpg", cv_bridge::toCvShare(msg, "bgr8")->image); 
// Mat image = cv::imread("test.jpg", IMREAD_COLOR); 
// cv::imshow("testinlezen", image); 
//cv::waitKey(0); 
//cv::destroyAllWindows(); 
} 

catch (cv_bridge::Exception& e){ 

ROS_ERROR("kan niet converten van '%s' to 'bgr8' .", msg->encoding.c_str()); 
} 

} 

  

/*static const std::string OPENCV_WINDOW = "Image window"; 

  
class ImageConverter 

{ 
ros::NodeHandle nh_; 
image_transport::ImageTransport it_; 
image_transport::Subscriber image_sub_; 
image_transport::Publisher image_pub_; 

 public: 
ImageConverter() 
: it_(nh_) 
{ 

// Subscrive to input video feed and publish output video feed 
image_sub_ = it_.subscribe("/camera/image_raw", 1, 
&ImageConverter::imageCb, this); 
image_pub_ = it_.advertise("/image_converter/output_video", 1); 

  
cv::namedWindow(OPENCV_WINDOW); 
} 
 ~ImageConverter() 
{ 
cv::destroyWindow(OPENCV_WINDOW); 
} 

  
void imageCb(const sensor_msgs::ImageConstPtr& msg) 
{ 
cv_bridge::CvImagePtr cv_ptr; 

try 
{ 
cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 

} 
catch (cv_bridge::Exception& e) 
{ 
ROS_ERROR("cv_bridge exception: %s", e.what()); 
return; 

} 
 
// Draw an example circle on the video stream 
if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) 
cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0)); 

 
// Update GUI Window 
cv::imshow(OPENCV_WINDOW, cv_ptr->image); 
cv::waitKey(3); 

// Output modified video stream 
image_pub_.publish(cv_ptr->toImageMsg()); 
} 
};*/ 

 
 

int main(int argc, char **argv) 
{ 
ros::init(argc, argv, "pcl_processor"); 
ros::init(argc, argv, "image_converter"); 
ros::NodeHandle nh; 

 //cv::namedWindow("view"); 

//image_transport::ImageTransport it(nh); 

//image_transport::Subscriber sub = it.subscribe("camera/color/image_raw", 1, imageCallback); 

//create ros wrapper object 

PclProcessorRos pcl_processor_ros; 

	ROS_INFO("Ready to calculate pose. test");
	ros::spin();
	// PclProcessorRos::CalculateObjectpose();
	return 0;
}



PclProcessorRos::PclProcessorRos() :
    nh_(ros::this_node::getName())
{
  uint32_t queue_size = 1;
	filtered_pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("pcl_cloud_filtered", queue_size);
	capture_pointcloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("pcl_cloud_capture", queue_size);
	ROS_INFO("1 node");
	calculate_objectpose_service = nh_.advertiseService("calculate_object_pose", &PclProcessorRos::CalculateObjectpose, this);
}

PclProcessorRos::~PclProcessorRos()
{
ROS_INFO("2 node");
}

//void PclProcessorRos::process(){
//}

bool PclProcessorRos::CalculateObjectpose(pcl_processor_msgs::CalculateObjectposeFromPointcloud::Request  &request,
         pcl_processor_msgs::CalculateObjectposeFromPointcloud::Response &response)
{

  ROS_INFO("3 node");

	sensor_msgs::PointCloud2 input_cloud;
	{
		sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth/color/points", ros::Duration(10));
	  if (msg == NULL){
				ROS_INFO("No point clound messages received");
				response.success = false;
				return false;
		}
	  else
	      input_cloud = *msg;
		  ROS_INFO("input_cloud ingesteld");
	}

	capture_pointcloud_pub.publish(input_cloud);

	{
  	pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;
	  pcl_conversions::toPCL(input_cloud, *cloud2);  // From ROS-PCL to PCL2

	  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::fromPCLPointCloud2(*cloud2, *pcl_cloud); // From PCL2 to PCL
		pcl_processor.setInputCloud(pcl_cloud);
	}

	pcl_processor.process();
	filtered_pcl_cloud = pcl_processor.getFilteredCloud();

	{
		pcl::PCLPointCloud2* filtered_cloud2 = new pcl::PCLPointCloud2;
		pcl::toPCLPointCloud2(*filtered_pcl_cloud, *filtered_cloud2); // From PCL to PCL2

	  /* initialaize pointcloud */
	  ROS_INFO("Create filtered ROS cloud");

	  sensor_msgs::PointCloud2 filtered_ros_cloud;
	  pcl::toROSMsg(*filtered_pcl_cloud, filtered_ros_cloud);

	  filtered_ros_cloud.header.frame_id = input_cloud.header.frame_id;

	  /* publish point cloud */
	  ROS_INFO("Publish filtered cloud");
	  filtered_pointcloud_pub.publish(filtered_ros_cloud);
	}

	{
		geometry_msgs::Vector3 object_position = pcl_processor.getObjectPosition();

		geometry_msgs::TransformStamped static_transformStamped;

		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfListener(tfBuffer);

		static tf2_ros::StaticTransformBroadcaster static_broadcaster;
		ROS_INFO("add static transform to object");
		/* add static transform to object */
		static_transformStamped.header.stamp = ros::Time::now();
		static_transformStamped.header.frame_id = "camera_depth_optical_frame";
		static_transformStamped.child_frame_id = "object_to_grasp";

		static_transformStamped.transform.translation = object_position;
		static_transformStamped.transform.rotation.x = 0;
		static_transformStamped.transform.rotation.y = 0;
		static_transformStamped.transform.rotation.z = 0;
		static_transformStamped.transform.rotation.w = 1.0;
		static_broadcaster.sendTransform(static_transformStamped);

		/* wait a while */
		ros::Duration(1.0).sleep();
		ROS_INFO("get object pose relative to world ");
		/* get object pose relative to world */
		try{
			geometry_msgs::TransformStamped transformStamped;
			geometry_msgs::PoseStamped poseStamped;
			poseStamped.pose.orientation.x = 0;
			poseStamped.pose.orientation.y = 0;
			poseStamped.pose.orientation.z = 0;
			poseStamped.pose.orientation.w = 1.0;
			transformStamped = tfBuffer.lookupTransform("world", "object_to_grasp", ros::Time(0));
			tf2::doTransform(poseStamped, poseStamped, transformStamped); // object_to_grasp is the PoseStamped I want to transform

			response.success = true;
			response.object_pose = poseStamped;

		}
		catch (tf2::TransformException &ex) {
			ROS_ERROR("Error lookupTransform.");
			response.success = false;
		}
	}

	ROS_INFO("CalculateObjectposeFromPointcloud exit");
	/* return object pose in response*/
	return true;
}
