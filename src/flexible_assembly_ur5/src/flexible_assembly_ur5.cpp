/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

///////////////https://github.com/jmiseikis/ur5_inf3480/blob/master/src/inf3480_move_robot.cpp
#include <signal.h>

#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include "config_parser/ParseConfig.h"
#include "config_parser/AddTimestamp.h"
#include "flexible_assembly_msgs/Assembly.h"
#include "flexible_assembly_msgs/Component.h"
#include "flexible_assembly_msgs/Config.h"
#include "flexible_assembly_msgs/Cords.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "geometry_msgs/PoseStamped.h"

//#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <visualization_msgs/Marker.h>

volatile sig_atomic_t stop;
int vacuumValue;

void inthand(int signum)
{
  stop = 1;
}

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

std::vector<double> zeroPose = {0, 0, 0, 0, 0, 0};
std::vector<double> homePose = {0, -1.57, 0, -1.57, 0, 0};
std::vector<double> prePickPose = {-1, -1.57, 0, -1.57, 0, 0};
std::vector<double> aboveComponents = {0.935846, -1.2205, 1.6568, -2.0302, -1.5809, 0};
std::vector<double> abovePlaceComponent = {0.012849807739257812, -1.106103245412008, -3.5587941304981996e-05, -0.5153783003436487, -1.5915849844561976, 0};//{0, -1.495131794606344, -0.03250343004335576, -0.8393414656268519, -1.6193788687335413, 0};
std::vector<double> placeComponentCircle = {0.894218921661377, -1.474051300679342, -0.029962841664449513, -1.0084355513202112, -1.6130526701556605, -0.02975780168642217};


//rectangle pose:
/*
header: 
  seq: 0
  stamp: 1642417594.072469532
  frame_id: base_link
pose: 
  position: 
    x: 0.371538
    y: 0.0968314
    z: 0.359131
  orientation: 
    x: -0.485076
    y: 0.499509
    z: -0.519288
    w: 0.495511
*/
//square
/*
header: 
  seq: 0
  stamp: 1642417743.323867500
  frame_id: base_link
pose: 
  position: 
    x: 0.292322
    y: 0.112182
    z: 0.353673
  orientation: 
    x: -0.52651
    y: 0.4639
    z: -0.487237
    w: 0.519793
*/
//circle
/*
header: 
  seq: 0
  stamp: 1642418083.145825700
  frame_id: base_link
pose: 
  position: 
    x: 0.294981
    y: 0.0953341
    z: 0.357182
  orientation: 
    x: -0.527232
    y: 0.478481
    z: -0.496081
    w: 0.496976

*/

enum States
{
  IDLE = 0,
  INITIALIZE = 1,
  READ_CONFIG = 2,
  PICK_ARDUINO = 3,
  REQUEST_COMPONENT = 4,
  REQUEST_PLACE_POSE = 5,
  ROTATE_ARDUINO = 6,
  PICK_COMPONENT = 7,
  PLACE_COMPONENT = 8,
  PLACE_ARDUINO = 9,
  DEINITIALIZE = 10,
  SHUTDOWN = 11,
  ERROR = 12,
  ASSEMBLE = 13,
  TEST = 100
};
States currentState;

bool initialize()
{
  //initialize AI
  //initialize camera
  //initialize ur4
  return false;
}

bool readConfig(ros::ServiceClient client, flexible_assembly_msgs::Assembly &assemblyConfig)
{
  config_parser::ParseConfig srv;
  srv.request.filepath = "/home/student/flexibleassembly_ws/config.xml";
  if (client.call(srv))
  {
    std::cout << "::: Components to assemble: " << std::endl;
    for (auto const &component : srv.response.config.assembly.components)
    {
      std::cout << "::: " << component.color << " " << component.shape << std::endl;
    }
    assemblyConfig = srv.response.config.assembly;

    std::cout << srv.response.config.assembly << std::endl;
    return 1;
    //ROS_INFO("config: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service config_parser");
    return 0;
  }
}

bool pickArduino()
{

  //do something
  return 0;
}

bool requestComponent()
{
  //do something
  // publish /request_component
  //ros::Publisher request_component_pub = n.advertise<std_msgs::Bool>("/request_component", 1, true);

  // subscribe & wait for message.
  // /detections Cords.

  return 0;
}

bool requestPlacePose()
{
  //do something
  return 0;
}

bool RotateArduino()
{
  //do something
  return 0;
}

bool PickComponent()
{
  //do something
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // move_group_interface.setStartState(*move_group_interface.getCurrentState());
  // move_group_interface.setJointValueTarget(aboveComponents);
  // if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  // {
  //   std::cout << "planning unsuccesfull" << std::endl;
  // }
  // else
  // {
  //   if (!move_group_interface.execute(my_plan))
  //   {
  //     std::cout << " execute plan not finished";
  //   }
  //   else
  //   {
  //     std::cout << "!!! moved to homePose !!!" << std::endl;
  //   }
  // }
  return 0;
}

bool placeComponent()
{
  //do something
  return 0;
}

bool placeArduino()
{
  //do something
  return 0;
}

bool deinitialize()
{
  //do something
  return 0;
}

bool assemble()
{
  //do something
  return 0;
}

double roundUp(double value, int decimal_places)
{
  const double multiplier = std::pow(10.0, decimal_places);
  return std::ceil(value * multiplier) / multiplier;
}

void printJointPositions(std::vector<double> joint_positions)
{
  for (int i = 0; i < joint_positions.size(); i++)
  {
    std::cout << "current joint " << i << " position: " << roundUp(joint_positions[i], 4) << std::endl;
  }
}

bool robotIsatPose(std::vector<double> current_joint_positions, std::vector<double> expected_joint_positions)
{
  int flag = 0;
  for (int i = 0; i < current_joint_positions.size(); i++)
  {
    double current_joint_position_rounded = roundUp(current_joint_positions[i], 4);
    double expected_joint_position_rounded = roundUp(expected_joint_positions[i], 4);

    double deviation = abs(current_joint_position_rounded - expected_joint_position_rounded);
    if (!(deviation < 0.05))
    {
      std::cout << "Joint " << i << ": current_joint_position_rounded: " << current_joint_position_rounded << std::endl;
      std::cout << "Joint " << i << ": expected_joint_position_rounded: " << expected_joint_position_rounded << std::endl;
      std::cout << "Joint " << i << ", deviation: " << deviation << std::endl;
      flag = 1;
    }
  }
  if (flag)
  {
    return false;
  }
  return true;
}

visualization_msgs::Marker getMarker(std::string frame_id, geometry_msgs::Pose marker_pose)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  // Set our initial shape type to be a cube
  marker.type = visualization_msgs::Marker::SPHERE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose = marker_pose;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(15);

  return marker;
}

std::vector<moveit_msgs::CollisionObject> getUR4CollisionObject(std::string frame_id)
{
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;

  /* The id of the object is used to identify it. */
  collision_object.id = "box1";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.25;
  primitive.dimensions[1] = 0.10;
  primitive.dimensions[2] = 0.02;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.35;
  box_pose.position.y = 0.11;
  box_pose.position.z = 0.32;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  return collision_objects;
}

std::vector<moveit_msgs::CollisionObject> getCollisionObjects(std::string frame_id)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  //SHAPE OBJECT
  //add objects to scene
  // moveit_msgs::CollisionObject collision_object;
  // collision_object.header.frame_id = frame_id; //move_group_interface.getPlanningFrame();
  // collision_object.id = "table";

  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[0] = 1.0;
  // primitive.dimensions[1] = 1.0;
  // primitive.dimensions[2] = 0.10;

  // geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = 0;
  // box_pose.position.y = 0;
  // box_pose.position.z = 0;

  // collision_object.primitives.push_back(primitive);
  // collision_object.primitive_poses.push_back(box_pose);
  // collision_object.operation = collision_object.ADD;

  // collision_objects.push_back(collision_object);

  //WORLD MESH OBJECT
  moveit_msgs::CollisionObject worldObj;
  shape_msgs::Mesh world_mesh;
  shapes::ShapeMsg world_mesh_msg;
  shapes::Mesh *m = shapes::createMeshFromResource("package://ur3_with_vacuum_tool_moveit_config/meshes/World.STL");
  shapes::constructMsgFromShape(m, world_mesh_msg);
  world_mesh = boost::get<shape_msgs::Mesh>(world_mesh_msg);

  worldObj.meshes.resize(1);
  worldObj.meshes[0] = world_mesh;
  worldObj.mesh_poses.resize(1);
  worldObj.header.frame_id = frame_id;
  worldObj.id = "world";
  worldObj.mesh_poses[0].position.x = -0.15;
  worldObj.mesh_poses[0].position.y = -0.35;
  worldObj.mesh_poses[0].position.z = -0.035;
  worldObj.mesh_poses[0].orientation.w = 0.0;
  worldObj.mesh_poses[0].orientation.x = 0.0;
  worldObj.mesh_poses[0].orientation.y = 0.0;
  worldObj.mesh_poses[0].orientation.z = 0.0;

  worldObj.meshes.push_back(world_mesh);
  worldObj.mesh_poses.push_back(worldObj.mesh_poses[0]);
  worldObj.operation = worldObj.ADD;
  collision_objects.push_back(worldObj);

  return collision_objects;
}

void visionPoseCallback(const geometry_msgs::PoseStamped &msg)
{
  //std::string place_pose = string(msg->data);
  //ROS_INFO("place_pose: [%s]", place_pose);
  std::cout << msg << std::endl;
}

void stateRequestCallback(const std_msgs::UInt8::ConstPtr &msg)
{
  int requestedState = int(msg->data);
  ROS_INFO("Requested state: [%i]", States(requestedState));

  if (currentState != IDLE)
  {
    ROS_WARN("Unable to activate requested state, wait untill the robot is idle.");
  }
  else
  {
    currentState = States(requestedState);
  }
}

void vacuumValueCallback(const std_msgs::UInt16::ConstPtr &msg)
{
  int receivedVacuumValue = int(msg->data);
  //ROS_INFO("Received vacuum value: [%i]", receivedVacuumValue);

  vacuumValue = receivedVacuumValue;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flexible_assembly");
  ros::NodeHandle n;

  currentState = INITIALIZE;

  //pub sub
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  ros::Publisher display_pub = n.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  ros::Publisher ur4_state_pub = n.advertise<std_msgs::UInt8>("/UR4/UR4_state", 0);
  ros::Publisher pick_arduino_pub = n.advertise<std_msgs::UInt8>("/UR4/UR4_state", 0);
  ros::Publisher request_component_pub = n.advertise<std_msgs::Bool>("/request_component", 0);
  ros::Publisher pomp_state_pub = n.advertise<std_msgs::Bool>("/UR5/Gripper_Pomp", 0);
  ros::Publisher request_place_pose = n.advertise<std_msgs::String>("/request_place_pose", 0);
  //ros::Publisher vacuum_state_pub = n.advertise<std_msgs::Bool>("/UR5/Gripper_Vacuum", 0);

  ros::Subscriber vacuum_value_sub = n.subscribe("/UR5/Gripper_Vacuum", 1, vacuumValueCallback);
  ros::Subscriber state_request_sub = n.subscribe("/UR5/state_request", 1, stateRequestCallback);
  ros::Subscriber vision_pose_sub = n.subscribe("/vision_pose_response", 1, visionPoseCallback);

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  //config
  ros::ServiceClient client = n.serviceClient<config_parser::ParseConfig>("config_parser");
  flexible_assembly_msgs::Assembly assemblyConfig;

  flexible_assembly_msgs::Cords receivedComponentInfo;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  //enable pomp & vacuum
  std_msgs::Bool pomp_state_msg;
  pomp_state_msg.data = false;
  pomp_state_pub.publish(pomp_state_msg);

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  //moveit::planning_interface::MoveGroupInterface *move_group_interface;
  //move_group_interface = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO_NAMED("Flexible_assembly", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  ROS_INFO_NAMED("Flexible_assembly", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("Flexible_assembly", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  std::cout << std::endl
            << std::endl;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::RobotStatePtr current_robot_state = move_group_interface.getCurrentState();
  //set moveit params
  //move_group_interface.setPlannerId("RRTConnect");
  //move_group_interface.setNumPlanningAttempts(50);
  //move_group_interface.setPlanningTime(50.0);
  move_group_interface.allowReplanning(true);

  move_group_interface.setMaxVelocityScalingFactor(0.1);
  move_group_interface.setMaxAccelerationScalingFactor(0.1);

  //add collision objects to scene
  std::vector<moveit_msgs::CollisionObject> collision_objects = getCollisionObjects(move_group_interface.getPlanningFrame());
  planning_scene_interface.addCollisionObjects(collision_objects);
  sleep(1.0);

  //get current joint positions
  std::vector<double> current_joint_positions;
  current_robot_state->copyJointGroupPositions(joint_model_group, current_joint_positions);

  //check if robot is at expected homePose
  if (!robotIsatPose(current_joint_positions, homePose))
  {
    ROS_ERROR("Robot is not at the expected pose 'homePose', please move the robot manually to the expected pose & restart.");
    //ros::shutdown();
  }
  else
  {
    currentState = IDLE;
  }

  //create rectangle pose here (not possible globally?)
  geometry_msgs::PoseStamped component_place_pose;
  signal(SIGINT, inthand);

  float radian;
  flexible_assembly_msgs::Component componentToPick;

  int previousState = currentState;
  ROS_INFO("Current state: [%i]", currentState);

  // std::cout << "::::::::::::::::::::" << std::endl;
  // std::cout << move_group_interface.getCurrentPose() << std::endl;
  // return 0;

  while (!stop)
  {
    if (currentState != previousState)
    {
      previousState = currentState;
      ROS_INFO("Current state: [%i]", currentState);
    }

    switch (currentState)
    {
    case IDLE: //0
      // do nothing and chill
      break;
    case INITIALIZE: //1
    {
      ROS_INFO("CASE INITIALIZING");
      //initialize();
      //publish
      std_msgs::UInt8 msg;
      msg.data = States(INITIALIZE);
      ur4_state_pub.publish(msg);
      //for(int i = 0; i < 500000; i++);

      //subscribe to get response
      boost::shared_ptr<std_msgs::Bool const> initialize_ur4_response;
      initialize_ur4_response = ros::topic::waitForMessage<std_msgs::Bool>("/UR4/UR4_state_response", ros::Duration(30));
      if (!initialize_ur4_response)
      {
        ROS_WARN("initialize_ur4_response false, or timeout");
      }
      currentState = States(IDLE);
      break;
    }
    case READ_CONFIG: //2
    {
      ROS_INFO("CASE READ_CONFIG");
      readConfig(client, assemblyConfig);
      currentState = States(IDLE);
      break;
    }
    case PICK_ARDUINO: //3
    {
      ROS_INFO("CASE PICK_ARDUINO");
      //pickArduino();
      if (assemblyConfig.components.size() <= 0)
      {
        ROS_WARN("No config loaded, or no valid components to assemble.");
      }
      else
      {
        //publish
        std_msgs::UInt8 msg;
        msg.data = States(PICK_ARDUINO);
        pick_arduino_pub.publish(msg);

        //subscribe to get response
        boost::shared_ptr<std_msgs::Bool const> pick_arduino_response;
        pick_arduino_response = ros::topic::waitForMessage<std_msgs::Bool>("/UR4/UR4_state_response", ros::Duration(30));
        if (!pick_arduino_response)
        {
          ROS_WARN("pick_arduino_response false, or timeout");
        }
      }
      currentState = States(IDLE);
      break;
    }
    case REQUEST_COMPONENT: //4
    {
      ROS_INFO("CASE REQUEST_COMPONENT");
      //requestComponent();
      //publish
      std_msgs::Bool msg;
      msg.data = true;
      request_component_pub.publish(msg);

      //subscribe to get response
      boost::shared_ptr<flexible_assembly_msgs::Cords const> request_component_response;
      request_component_response = ros::topic::waitForMessage<flexible_assembly_msgs::Cords>("/detections", ros::Duration(30));
      if (!request_component_response)
      {
        ROS_WARN("request_component_response false, or timeout");
      }
      else
      {
        receivedComponentInfo = *request_component_response;
        //std::cout << request_component_response << std::endl;
        for (auto const &component : receivedComponentInfo.cords)
        {
          std::cout << "::: " << component.color << " " << component.shape << std::endl;
        }
      }

      currentState = States(IDLE);
      break;
    }
    case REQUEST_PLACE_POSE: //5
    {
      ROS_INFO("CASE REQUEST_PLACE_POSE");
      //requestComponent();

      //publish requested component pose
      std_msgs::String msg;
      msg.data = componentToPick.shape;
      ROS_INFO("request_place_pose msg: %s", msg.data);
      request_place_pose.publish(msg);
      //TODO: wait for reply true/false
      sleep(2);

      //publish
      std_msgs::UInt8 request_place_pose_msg;
      request_place_pose_msg.data = States(REQUEST_PLACE_POSE);
      ur4_state_pub.publish(request_place_pose_msg);

      //subscribe to get response
      boost::shared_ptr<geometry_msgs::PoseStamped const> request_place_pose_response;
      request_place_pose_response = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/vision_pose_response", ros::Duration(30));
      if (!request_place_pose_response)
      {
        ROS_WARN("request_place_pose_response false, or timeout");
      }
      else
      {
        //TODO
        std::cout << request_place_pose_response << std::endl;
        component_place_pose = *request_place_pose_response;
      }

      currentState = States(IDLE);
      break;
    }
    case ROTATE_ARDUINO: //6
    {
      ROS_INFO("CASE ROTATE_ARDUINO");
      //RotateArduino();
      //publish
      std_msgs::UInt8 msg;
      msg.data = States(ROTATE_ARDUINO);
      pick_arduino_pub.publish(msg);

      //subscribe to get response
      boost::shared_ptr<std_msgs::Bool const> rotate_arduino_response;
      rotate_arduino_response = ros::topic::waitForMessage<std_msgs::Bool>("/UR4/UR4_state_response", ros::Duration(30));
      if (!rotate_arduino_response)
      {
        ROS_WARN("rotate_arduino_response false, or timeout");
      }
      else
      {
        std::cout << rotate_arduino_response << std::endl;
      }

      currentState = States(IDLE);
      break;
    }
    case PICK_COMPONENT: //7
    {
      ROS_INFO("CASE PICK_COMPONENT");
      //PickComponent();
      //add collision object
      std::vector<moveit_msgs::CollisionObject> ur4CollisionObject = getUR4CollisionObject(move_group_interface.getPlanningFrame());
      planning_scene_interface.addCollisionObjects(ur4CollisionObject);
      sleep(10.0);

      //homePose: set jointvalues, plan, execute
      for (auto const &component : assemblyConfig.components)
      {
        if (component.timestamp == "")
        {
          componentToPick = component;
          break;
        }
      }

      geometry_msgs::PoseStamped poseToPick;
      for (auto const &receivedComponent : receivedComponentInfo.cords)
      {
        if (receivedComponent.shape == componentToPick.shape && receivedComponent.color == componentToPick.color)
        {
          std::cout << receivedComponent << std::endl;
          poseToPick.pose.position = receivedComponent.pose.position;
          radian = float(receivedComponent.radian);
          //componentToPickPose.pose.position = receivedComponent.pose.position;
          //componentToPickPose.pose.orientation = receivedComponent.pose.orientation;
          break;
        }
      }

      //abovepose:
      // header:
      //   seq: 0
      //   stamp: 1642581008.490092889
      //   frame_id: base_link
      // pose:
      //   position:
      //     x: 0.127733
      //     y: 0.360178
      //     z: 0.135741
      //   orientation:
      //     x: -0.66538
      //     y: 0.214758
      //     z: -0.226623
      //     w: 0.678078

      //move to safe pose above components
      //set jointvalues, plan, execute
      move_group_interface.setJointValueTarget(aboveComponents);
      if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        std::cout << "planning unsuccesfull" << std::endl;
      }
      else
      {
        if (!move_group_interface.execute(my_plan))
        {
          std::cout << " execute plan not finished";
        }
        else
        {
          std::cout << "!!! moved to aboveComponents !!!" << std::endl;
        }
      }

      //create pose with old orientation, and new pick position
      geometry_msgs::PoseStamped componentToPickPose = move_group_interface.getCurrentPose();
      geometry_msgs::PoseStamped safeAboveComponents = componentToPickPose;
      componentToPickPose.pose.position = poseToPick.pose.position;
      componentToPickPose.pose.orientation.x = -0.66538;
      componentToPickPose.pose.orientation.y = 0.214758;
      componentToPickPose.pose.orientation.z = -0.226623;
      componentToPickPose.pose.orientation.w = 0.678078;

      //pick
      move_group_interface.setStartState(*move_group_interface.getCurrentState());
      move_group_interface.setPoseTarget(componentToPickPose);
      if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        std::cout << "planning to componentToPickPose unsuccesfull" << std::endl;
      }
      else
      {
        if (!move_group_interface.execute(my_plan))
        {
          std::cout << " execute plan to componentToPickPose not finished";
        }
        else
        {
          std::cout << "!!! moved to componentToPickPose !!!" << std::endl;
        }
      }

      sleep(0.5);
      //enable pomp & vacuum
      pomp_state_msg.data = true;
      pomp_state_pub.publish(pomp_state_msg);

      //check vacuum

      //safe above components
      move_group_interface.setStartState(*move_group_interface.getCurrentState());
      move_group_interface.setPoseTarget(safeAboveComponents);
      if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        std::cout << "planning to safeAboveComponents unsuccesfull" << std::endl;
      }
      else
      {
        if (!move_group_interface.execute(my_plan))
        {
          std::cout << " execute plan to safeAboveComponents not finished";
        }
        else
        {
          std::cout << "!!! moved to safeAboveComponents !!!" << std::endl;
        }
      }

      //home
      //set jointvalues, plan, execute
      move_group_interface.setStartState(*move_group_interface.getCurrentState());
      move_group_interface.setJointValueTarget(homePose);
      if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        std::cout << "planning unsuccesfull" << std::endl;
      }
      else
      {
        if (!move_group_interface.execute(my_plan))
        {
          std::cout << " execute plan not finished";
        }
        else
        {
          std::cout << "!!! moved to homePose !!!" << std::endl;
        }
      }

      //collision_objects.back().REMOVE;
      //planning_scene_interface.removeCollisionObjects(ur4CollisionObject);
      currentState = States(IDLE);
      break;
    }
    case PLACE_COMPONENT: //8
    {
      ROS_INFO("CASE PLACE_COMPONENT");
      //placeComponent();

      //enable pomp & vacuum
      pomp_state_msg.data = true;
      pomp_state_pub.publish(pomp_state_msg);

      //abovePlaceComponent
      //set jointvalues, plan, execute
      std::cout << "adjusting J6 to radian: " << radian << std::endl;
      //abovePlaceComponent[5] = radian;

      move_group_interface.setStartState(*move_group_interface.getCurrentState());
      move_group_interface.setJointValueTarget(abovePlaceComponent);
      if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        std::cout << "planning unsuccesfull" << std::endl;
      }
      else
      {
        if (!move_group_interface.execute(my_plan))
        {
          std::cout << " execute plan not finished";
        }
        else
        {
          std::cout << "!!! moved to abovePlaceComponent !!!" << std::endl;
        }
      }

      //component_place_pose: set jointvalues, plan, execute
      move_group_interface.setStartState(*move_group_interface.getCurrentState());
      //component_place_pose.pose.orientation.z = move_group_interface.getCurrentPose().pose.orientation.z;
      //move_group_interface.setPoseTarget(component_place_pose);
      move_group_interface.setJointValueTarget(placeComponentCircle);
      if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        std::cout << "planning to placeComponentCircle unsuccesfull" << std::endl;
      }
      else
      {
        if (!move_group_interface.execute(my_plan))
        {
          std::cout << " execute plan to placeComponentCircle not finished";
        }
        else
        {
          std::cout << "!!! moved to placeComponentCircle !!!" << std::endl;
        }
      }

      //disable pomp & vacuum
      pomp_state_msg.data = false;
      pomp_state_pub.publish(pomp_state_msg);
      sleep(0.1);

      //component_place_pose: set jointvalues, plan, execute
      move_group_interface.setStartState(*move_group_interface.getCurrentState());
      move_group_interface.setJointValueTarget(homePose);
      if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        std::cout << "planning to homePose unsuccesfull" << std::endl;
      }
      else
      {
        if (!move_group_interface.execute(my_plan))
        {
          std::cout << " execute plan to homePose not finished";
        }
        else
        {
          std::cout << "!!! moved to homePose !!!" << std::endl;
        }
      }

      currentState = States(IDLE);
      break;
    }
    case PLACE_ARDUINO: //9
      ROS_INFO("CASE PLACE_ARDUINO");
      //placeArduino();
      currentState = States(IDLE);
      break;
    case DEINITIALIZE: //10
      ROS_INFO("CASE DEINITIALIZE");
      //deinitialize();
      currentState = States(IDLE);
      break;
    case SHUTDOWN: //11
      ROS_INFO("Shutdown state activated, node shutting down . . . ");
      ros::shutdown();
    case ERROR: //12
      ROS_INFO("CASE ERROR");
      // do nothing and chill
      break;
    case ASSEMBLE: //13
      ROS_INFO("CASE ASSEMBLE");
      // do nothing and chill
      break;
    case TEST: //100
    {
      //test 
      move_group_interface.setStartState(*move_group_interface.getCurrentState());
      move_group_interface.setJointValueTarget(placeComponentCircle);
      if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      {
        std::cout << "planning unsuccesfull" << std::endl;
      }
      else
      {
        if (!move_group_interface.execute(my_plan))
        {
          std::cout << " execute plan not finished";
        }
        else
        {
          std::cout << "!!! moved to placeComponentCircle !!!" << std::endl;
        }
      }
      currentState = States(IDLE);
      break;
    }

    default:
      ROS_ERROR("CurrentState is unknown");
    }
  }

  return 0;

  //prePickPose: set jointvalues, plan, execute
  move_group_interface.setJointValueTarget(prePickPose);
  if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    std::cout << "planning unsuccesfull" << std::endl;
  }
  else
  {
    if (!move_group_interface.execute(my_plan))
    {
      std::cout << " execute plan not finished" << std::endl;
    }
    else
    {
      std::cout << "!!! moved to prePickPose !!!" << std::endl;
    }
  }

  ///
  geometry_msgs::PoseStamped robot_pose = move_group_interface.getCurrentPose();

  //HomePose: set jointvalues, plan, execute
  move_group_interface.setJointValueTarget(homePose);
  if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    std::cout << "planning unsuccesfull" << std::endl;
  }
  else
  {
    if (!move_group_interface.execute(my_plan))
    {
      std::cout << " execute plan not finished";
    }
    else
    {
      std::cout << "!!! moved to homePose !!!" << std::endl;
    }
  }

  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  geometry_msgs::PoseStamped target_pose1;
  target_pose1.pose = robot_pose.pose;
  move_group_interface.clearPoseTarget();

  //move_group_interface.setEndEffectorLink("tool0");
  move_group_interface.setPoseTarget(target_pose1.pose);

  //SHOW MARKER AT TARGET POSE
  visualization_msgs::Marker marker = getMarker(move_group_interface.getPlanningFrame(), target_pose1.pose);
  marker_pub.publish(marker);
  sleep(3);
  //

  if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    std::cout << "planning unsuccesfull" << std::endl;
  }
  else
  {
    if (!move_group_interface.execute(my_plan))
    {
      std::cout << " execute plan not finished";
    }
    else
    {
      std::cout << "!!! moved to prePickPose !!!" << std::endl;
    }
  }

  //homePose: set jointvalues, plan, execute
  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  move_group_interface.setJointValueTarget(homePose);
  if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    std::cout << "planning unsuccesfull" << std::endl;
  }
  else
  {
    if (!move_group_interface.execute(my_plan))
    {
      std::cout << " execute plan not finished";
    }
    else
    {
      std::cout << "!!! moved to homePose !!!" << std::endl;
    }
  }

  //Remove objects from scene
  std::vector<std::string> object_ids;
  for (auto collision_object : collision_objects)
  {
    object_ids.push_back(collision_object.id);
  }
  planning_scene_interface.removeCollisionObjects(object_ids);
  sleep(3.0);

  ros::shutdown();
  return 0;
}
