#include "poses.h"

  JointPose::JointPose(std::string name){ //}, std::vector<float> joint_poses){
    poseName = name;
    //joint_positions = joint_poses
  }

  void JointPose::printinfo()
  {
    std::cout << "pose name: " << poseName << std::endl;
    for (double i : joint_positions)
    {
      //std::cout << fmt::format("Joint[{}] = {}", i, joint_positions[i]);
      std::cout << "joint " << i << std::endl;
    }
  }
