#ifndef POSES_H
#define POSES_H

class JointPose
{
private:
    std::string poseName;
    std::vector<float> joint_positions;

public:
  JointPose(std::string name); //, std::vector<float> joint_poses);
  void printinfo();

};

#endif



