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

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <visualization_msgs/Marker.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

std::vector<double> zeroPose = {0, 0, 0, 0, 0, 0};
std::vector<double> homePose = {0, -1.57, 0, -1.57, 0, 0};
std::vector<double> prePickPose = {-1, -1.57, 0, -1.57, 0, 0};

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
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(15);

  return marker;
}
std::vector<moveit_msgs::CollisionObject> getCollisionObjects(std::string frame_id)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  // //SHAPE OBJECT
  // //add objects to scene
  // moveit_msgs::CollisionObject collision_object;
  // collision_object.header.frame_id = frame_id; //move_group_interface.getPlanningFrame();
  // collision_object.id = "table";

  // shape_msgs::SolidPrimitive primitive;
  // primitive.type = primitive.BOX;
  // primitive.dimensions.resize(3);
  // primitive.dimensions[0] = 0.10;
  // primitive.dimensions[1] = 0.10;
  // primitive.dimensions[2] = 0.10;

  // geometry_msgs::Pose box_pose;
  // box_pose.orientation.w = 1.0;
  // box_pose.position.x = 0.15;
  // box_pose.position.y = 0.15;
  // box_pose.position.z = 0.2;

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
  worldObj.mesh_poses[0].position.x = -0.20;
  worldObj.mesh_poses[0].position.y = -0.15;
  worldObj.mesh_poses[0].position.z = -0.025;
  worldObj.mesh_poses[0].orientation.w = 0.0;
  worldObj.mesh_poses[0].orientation.x = 0.0;
  worldObj.mesh_poses[0].orientation.y = 0.0;
  worldObj.mesh_poses[0].orientation.z = 0.0;

  worldObj.meshes.push_back(world_mesh);
  worldObj.mesh_poses.push_back(worldObj.mesh_poses[0]);
  worldObj.operation = worldObj.ADD;
  // worldObj.meshes.push_back(mesh);
  // worldObj.mesh_poses.push_back(worldObj.mesh_poses[0]);
  // worldObj.operation = worldObj.ADD;
  collision_objects.push_back(worldObj);

  return collision_objects;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flexible_assembly");
  ros::NodeHandle node_handle;

  //pub sub
  ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  ROS_INFO_NAMED("Flexible_assembly", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  ROS_INFO_NAMED("Flexible_assembly", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("Flexible_assembly", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
  std::cout << std::endl
            << std::endl;

  /////////////////////////////////////////////////////////////////////////////////
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
  //set moveit params
  move_group_interface.setPlannerId("OMPL");
  move_group_interface.allowReplanning(true);
  move_group_interface.setPlanningTime(5.0);
  move_group_interface.setNumPlanningAttempts(10);
  move_group_interface.setMaxVelocityScalingFactor(1.0);
  move_group_interface.setMaxAccelerationScalingFactor(1.0);

  //add collision objects to scene
  std::vector<moveit_msgs::CollisionObject> collision_objects = getCollisionObjects(move_group_interface.getPlanningFrame());
  planning_scene_interface.addCollisionObjects(collision_objects);
  sleep(5.0);

  //get current joint positions
  std::vector<double> current_joint_positions;
  current_state->copyJointGroupPositions(joint_model_group, current_joint_positions);

  //check if robot is at expected homePose
  if (!robotIsatPose(current_joint_positions, homePose))
  {
    std::cout << "Robot is not at the expected pose 'homePose', please move the robot manually to the expected pose & restart." << std::endl;
    ros::shutdown();
  }

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
      std::cout << " execute plan not finished";
    }
  }

  move_group_interface.setStartState(*move_group_interface.getCurrentState());
  geometry_msgs::Pose target_pose1; 
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.3;
  target_pose1.position.y = 0.3;
  target_pose1.position.z = 0.4;
  move_group_interface.setPoseTarget(target_pose1);

  //SHOW MARKER AT TARGET POSE
  visualization_msgs::Marker marker = getMarker(move_group_interface.getPlanningFrame(), target_pose1);
  marker_pub.publish(marker);
  //

  moveit::planning_interface::MoveItErrorCode ret = move_group_interface.plan(my_plan);
  std::cout << "ret: " << ret << std::endl;
  //int32 SUCCESS=1
  // int32 FAILURE=99999
  // int32 PLANNING_FAILED=-1
  // int32 INVALID_MOTION_PLAN=-2
  // int32 MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-3
  // int32 CONTROL_FAILED=-4
  // int32 UNABLE_TO_AQUIRE_SENSOR_DATA=-5
  // int32 TIMED_OUT=-6
  // int32 PREEMPTED=-7

  //if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  if (ret != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    std::cout << "planning unsuccesfull" << std::endl;
  }
  else
  {
    ROS_INFO("PLANNED< EXECUTION POSSIBLE");
    //move_group_interface.execute(my_plan);
  }

  //homePose: set jointvalues, plan, execute
  move_group_interface.setJointValueTarget(homePose);
  if (!move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    std::cout << "planning unsuccesfull" << std::endl;
  }
  else
  {
    move_group_interface.execute(my_plan);
  }

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
