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
#include <signal.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>

volatile sig_atomic_t stop;
int switch_pose = 0;
int Flexibleassembly_Received_State;
std::string sub_place_pose;

void inthand(int signum)
{
  stop = 1;
}
// %Tag(CALLBACK)%
void chatterCallback(const std_msgs::UInt8::ConstPtr &state_msg)
{
  ROS_INFO("I heard: [%i]", state_msg->data);
  Flexibleassembly_Received_State = state_msg->data;
}

void chatterCallbackPose(const std_msgs::String::ConstPtr &place_pose_msg)
{
  sub_place_pose = place_pose_msg->data;
  ROS_INFO("I heard: [%s]", sub_place_pose);
}

// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_processor");

  ros::NodeHandle node_handle;

  ros::Subscriber Flexibleassembly_Intialize = node_handle.subscribe("/UR4/UR4_state", 1, chatterCallback);
  ros::Subscriber vision_place_pose = node_handle.subscribe("/request_place_pose", 1, chatterCallbackPose);
  ros::Publisher vision_state = node_handle.advertise<std_msgs::Bool>("/vision_state_response", 1, true);
  ros::Publisher vision_pose = node_handle.advertise<geometry_msgs::PoseStamped>("/vision_pose_response", 1, true);

  // ros::Publisher robotiq_pub = node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("/UR4/Robotiq2FGripperRobotOutput", 1000);
  // ROS spinning must be running for the MoveGroupInterface to get information about the robot's state.
  // One way to do this is to start an AsyncSpinner  beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // %Tag(FILL_MESSAGE)%
  std_msgs::UInt8 state_msg;
  std_msgs::Bool vision_state_msg;
  //std_msgs::String place_pose_msg;
  geometry_msgs::PoseStamped p;
  volatile sig_atomic_t stop;
  int switch_pose = 0;

  signal(SIGINT, inthand);

  while (!stop)
  {
    switch (Flexibleassembly_Received_State)
    {
    case 0:
    {
      //idle
      ;
      break;
    }
    case 1:
    {
      //initialize
      vision_state_msg.data = 1;
      vision_state.publish(vision_state_msg);
      Flexibleassembly_Received_State = 0;
      break;
    }
    case 5:
    {
      ROS_INFO("CASE 5");
      std::cout << "in case 5 sub_place_pose = " << sub_place_pose << std::endl;
      if (sub_place_pose == "square")
      {
        switch_pose = 1;
        ROS_INFO("Square cords");
        p.header.frame_id = "base_link";
        p.header.seq = 0;
        ;

        //position
        p.pose.position.x = 0.292322;
        p.pose.position.y = 0.112182;
        p.pose.position.z = 0.353673;

        //orientation
        p.pose.orientation.x = -0.52651;
        p.pose.orientation.y = 0.4639;
        p.pose.orientation.z = -0.487237;
        p.pose.orientation.w = 0.519793;

        vision_pose.publish(p);
        switch_pose = 0;
      }
      else if (sub_place_pose == "rectangle")
      {
        switch_pose = 2;
        ROS_INFO("Rectangle cords");
        p.header.frame_id = "base_link";
        p.header.seq = 0;

        //position
        p.pose.position.x = 0.371538;
        p.pose.position.y = 0.0968314;
        p.pose.position.z = 0.359131;

        //orientation
        p.pose.orientation.x = -0.485076;
        p.pose.orientation.y = 0.499509;
        p.pose.orientation.z = -0.519288;
        p.pose.orientation.w = 0.495511;

        vision_pose.publish(p);
        switch_pose = 0;
      }
      else if (sub_place_pose == "circle")
      {
        switch_pose = 3;
        ROS_INFO("Circle cords");
        p.header.frame_id = "base_link";
        p.header.seq = 0;

        //position
        p.pose.position.x = 0.294981;
        p.pose.position.y = 0.0953341;
        p.pose.position.z = 0.357182;

        //orientation
        p.pose.orientation.x = -0.527232;
        p.pose.orientation.y = 0.478481;
        p.pose.orientation.z = -0.496081;
        p.pose.orientation.w = 0.496976;

        vision_pose.publish(p);
        switch_pose = 0;
      }

      // switch (switch_pose)
      // {
      //   case 0:{

      //   }
      //   case 1:
      //   {
      //     ROS_INFO("Square cords");
      //     p.header.frame_id = "base_link";
      //     p.header.seq = 0;;

      //     //position
      //     p.pose.position.x = 0.292322;
      //     p.pose.position.y = 0.112182;
      //     p.pose.position.z = 0.353673;

      //     //orientation
      //     p.pose.orientation.x = -0.52651;
      //     p.pose.orientation.y = 0.4639;
      //     p.pose.orientation.z = -0.487237;
      //     p.pose.orientation.w = 519793;

      //     vision_pose.publish(p);
      //     switch_pose = 0;
      //     break;
      //   }

      //   case 2:
      //   {
      //     ROS_INFO("Rectangle cords");
      //     p.header.frame_id = "base_link";
      //     p.header.seq = 0;

      //     //position
      //     p.pose.position.x = 0.371538;
      //     p.pose.position.y = 0.0968314;
      //     p.pose.position.z = 0.359131;

      //     //orientation
      //     p.pose.orientation.x = -0.485076;
      //     p.pose.orientation.y = 0.499509;
      //     p.pose.orientation.z = -0.519288;
      //     p.pose.orientation.w = 0.495511;

      //     vision_pose.publish(p);
      //     switch_pose = 0;
      //     break;
      //   }

      //   case 3:
      //   {
      //     ROS_INFO("Circle cords");
      //     p.header.frame_id = "base_link";
      //     p.header.seq = 0;

      //     //position
      //     p.pose.position.x = 0.294981;
      //     p.pose.position.y = 0.0953341;
      //     p.pose.position.z = 0.357182;

      //     //orientation
      //     p.pose.orientation.x = -0.527232;
      //     p.pose.orientation.y = 0.478481;
      //     p.pose.orientation.z = -0.496081;
      //     p.pose.orientation.w = 0.496976;

      //     vision_pose.publish(p);
      //     switch_pose = 0;
      //     break;
      //   }
      //   Flexibleassembly_Received_State = 0;

      //   default:
      //   {
      //     ROS_INFO("Default case");
      //     break;
      //   }
      // }
    }

    case 10:
    {
      //Deinitialize(robotiq_pub);
      Flexibleassembly_Received_State = 0;
      break;
    }
    }
  }
  ros::shutdown();
  return 0;
}
