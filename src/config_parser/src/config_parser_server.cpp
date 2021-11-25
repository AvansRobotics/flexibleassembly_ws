#include "ros/ros.h"
#include "config_parser/ParseConfig.h"
#include "config_parser/AddTimestamp.h"
#include "std_msgs/String.h"
#include <tinyxml.h>

bool parseConfig(config_parser::ParseConfig::Request &req,
                 config_parser::ParseConfig::Response &res)
{
  std::string filepath = req.filepath;
  ROS_INFO("config filepath: '%s'", filepath.c_str());

  TiXmlDocument doc;
  if(!doc.LoadFile(filepath))
  {
    std::cout << "can not parse " << filepath << std::endl;
    return false;
  }
  
  TiXmlElement* assemblyElement =  doc.RootElement ();
  TiXmlElement* assembly_id = assemblyElement->FirstChildElement("id");
  TiXmlElement* assembly_processed_timestamp = assemblyElement->FirstChildElement("processed_timestamp");
  TiXmlElement* assembly_base = assemblyElement->FirstChildElement("base");
  TiXmlElement* assembly_color = assemblyElement->FirstChildElement("color");
  
  std::cout << std::endl;
  std::cout << "assembly_id: " << assembly_id->GetText() << std::endl;
  std::cout << "assembly_processed_timestamp: " << assembly_processed_timestamp->GetText() << std::endl;
  std::cout << "assembly_base: " << assembly_base->GetText() << std::endl;
  std::cout << "assembly_color: " << assembly_color->GetText() << std::endl;



  // TiXmlElement *rootElement = doc.RootElement();                    // school element
  // Tixmlelement *classelement = rootelement > firstchildelement();   // class element
  // TiXmlElement *studentElement = classElement->FirstChildElement(); //Students

  // for (; studentElement != NULL; studentElement = studentElement->NextSiblingElement())
  // {
  //   Tixmlattribute *attributeofstudent = studentelement > firstattribute(); // get the name attribute of student
  //   for (; attributeOfStudent != NULL; attributeOfStudent = attributeOfStudent->Next())
  //   {
  //     cout << attributeOfStudent->Name() << " : " << attributeOfStudent->Value() << std::endl;
  //   }

  //   Tixmlelement *studentcontactelement = studentelement > firstchildelement(); // get the first contact information of the student

  //   for (; studentContactElement != NULL; studentContactElement = studentContactElement->NextSiblingElement())
  //   {
  //     string contactType = studentContactElement->Value();
  //     string contactValue = studentContactElement->GetText();
  //     cout << contactType << " : " << contactValue << std::endl;
  //   }
  // }
// }

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