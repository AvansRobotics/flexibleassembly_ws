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

#include <moveit_visual_tools/moveit_visual_tools.h>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

std::vector<double> zeroPose = {0, 0, 0, 0, 0, 0};
std::vector<double> homePose = {0, -1.57, 0, -1.57, 0, 0};

bool robotIsatPose(std::vector<double> current_joint_positions, std::vector<double> expected_joint_positions)
{
  for (int i : current_joint_positions)
  {
    if (!abs(current_joint_positions[i] - goal_positions[i]) < 0.005)
    {
      std::cout << "Robot is not at the expected pose, please move the robot manually to the expected pose." << std::endl;
      return false;
    }
  }
  return true;
}

bool addCollisionObjects()
{
  //add objects to scene
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();

  collision_object.id = "table";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 10;
  primitive.dimensions[1] = 10;
  primitive.dimensions[2] = 10;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface.addCollisionObjects(collision_objects);

  //std::Vector3d b(0.001, 0.001, 0.001);
  // moveit_msgs::CollisionObject co;
  // co.id = "table";
  // shapes::Mesh *m = shapes::createMeshFromResource("package://ur3_with_vacuum_tool_moveit_config/meshes/World.STL");
  // ROS_INFO("World mesh loaded");

  // shape_msgs::Mesh mesh;
  // shapes::ShapeMsg mesh_msg;
  // shapes::constructMsgFromShape(m, mesh_msg);
  // mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

  // //co.meshes.resize(1);
  // //co.mesh_poses.resize(1);
  // co.meshes[0] = mesh;
  // co.header.frame_id = move_group_interface.getPlanningFrame();
  // co.mesh_poses[0].position.x = 0;
  // co.mesh_poses[0].position.y = 0;
  // co.mesh_poses[0].position.z = 0.0;
  // co.mesh_poses[0].orientation.w = 0.0;
  // co.mesh_poses[0].orientation.x = 0.0;
  // co.mesh_poses[0].orientation.y = 0.0;
  // co.mesh_poses[0].orientation.z = 0.0;

  // co.meshes.push_back(mesh);
  // co.mesh_poses.push_back(co.mesh_poses[0]);
  // co.operation = co.ADD;
  // std::vector<moveit_msgs::CollisionObject> vec;
  // vec.push_back(co);
  // ROS_INFO("World added into the world");
  // planning_scene_interface.addCollisionObjects(vec);
  // sleep(5.0);
  ////////////////////////////
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;

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



  /////////////////////////////////////////////////////////////////////////////////
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

  //get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  std::cout << "joint_group_positions before:" << std::endl;
  for (double i : joint_group_positions)
    std::cout << i << std::endl;

  //Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = -1.3825; // -1/6 turn in radians
  joint_group_positions[1] = -1.7265;
  joint_group_positions[2] = -1.8083;
  joint_group_positions[3] = -1.1071;
  joint_group_positions[4] = 1.5708;
  joint_group_positions[5] = 0.4221;

  // std::vector<float> joint_poses{-1.3825,-1.7265,-1.8083,-1.1071,1.5708,0.4221};

  joint_group_positions[0] = joint_group_positions[0] + 0.2;

  std::cout << "joint_group_positions after:" << std::endl;
  for (double i : joint_group_positions)
    std::cout << i << std::endl;

  move_group_interface.setJointValueTarget(joint_group_positions);

  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
  move_group_interface.setMaxVelocityScalingFactor(0.05);
  move_group_interface.setMaxAccelerationScalingFactor(0.05);

  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    move_group_interface.execute(my_plan);
  }
  else
  {
    std::cout << "unsuccesfull" << std::endl;
  }

  std::cout << "*********************************Done" << std::endl;

  ros::shutdown();
  return 0;
}
