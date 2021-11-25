#include "ros/ros.h"
#include "config_parser/ParseConfig.h"
#include "config_parser/AddTimestamp.h"
#include "std_msgs/String.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "config_parser_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<config_parser::ParseConfig>("config_parser");
  config_parser::ParseConfig srv;
  srv.request.filepath = "/home/washichi/flexibleassembly_ws/config.xml";
  if (client.call(srv))
  {
    //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    ;
  }
  else
  {
    ROS_ERROR("Failed to call service config_parser");
    return 1;
  }

  return 0;
}