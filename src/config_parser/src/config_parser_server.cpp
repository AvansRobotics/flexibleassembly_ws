#include "ros/ros.h"
#include "config_parser/ParseConfig.h"
#include "config_parser/AddTimestamp.h"
#include "std_msgs/String.h"
#include "flexible_assembly_msgs/Config.h"
#include "flexible_assembly_msgs/Assembly.h"
#include "flexible_assembly_msgs/Config.h"
#include <tinyxml.h>

bool parseConfig(config_parser::ParseConfig::Request &req, config_parser::ParseConfig::Response &res)
{
  std::string filepath = req.filepath;

  TiXmlDocument doc;
  if (!doc.LoadFile(filepath))
  {
    std::cout << "can not parse " << filepath << std::endl;
    return false;
  }

  flexible_assembly_msgs::Config config_msg;
  config_msg.header.stamp = ros::Time::now();

  TiXmlElement* assemblyElement = doc.RootElement();
  std::string assembly_id = assemblyElement->FirstChildElement("id")->GetText();
  std::string assembly_timestamp = assemblyElement->FirstChildElement("timestamp")->GetText();
  std::string assembly_base = assemblyElement->FirstChildElement("base")->GetText();
  std::string assembly_color = assemblyElement->FirstChildElement("color")->GetText();

  flexible_assembly_msgs::Assembly assembly_msg;
  assembly_msg.id = assembly_id;
  assembly_msg.timestamp = assembly_timestamp;
  assembly_msg.base = assembly_base;
  assembly_msg.color = assembly_color;

  for (TiXmlNode *componentNode = assemblyElement->FirstChild("component"); componentNode; componentNode = componentNode->NextSibling("component"))
  {
    std::string component_id = componentNode->FirstChildElement("id")->GetText();
    std::string component_shape = componentNode->FirstChildElement("shape")->GetText();
    std::string component_color = componentNode->FirstChildElement("color")->GetText();

    flexible_assembly_msgs::Component component_msg;
    component_msg.id = component_id;
    component_msg.shape = component_shape;
    component_msg.color = component_color;
    assembly_msg.components.push_back(component_msg);
  }
  config_msg.assembly = assembly_msg;
  res.config = config_msg;

  ROS_INFO("configfile '%s' parsed.", filepath.c_str());
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "config_parser_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("config_parser", parseConfig);
  ROS_INFO("Ready to parse config");
  ros::spin();

  return 0;
}