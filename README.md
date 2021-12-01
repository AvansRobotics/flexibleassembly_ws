# FlexibleAssembly
Collaborative Flexible assembling 
======

<p align="center">
  <img src="docs/images/concept.png" />
</p>

This repository provides the ROS packages for the project 'Collaborative Flexibe Assembling', which is part of 'Future Logistics'. 
This project has been realised as part of the minor Robotics at Avans Breda.
The recommend operating environment is on Ubuntu 18.04 with ROS Melodic. So far These packages haven't been tested in other environment.

### Installation on Ubuntu 18.04 + ROS Melodic

**Make sure your system is up to date and ros is installed**

**Install this repository from Source:**
This repo is the complete workspace (flexibleassembly_ws) without the recursive sub-modules.
Clone this repo into the folder where you want to initialize your ws, for examle:
```sh
$ cd ~/
$ git clone https://github.com/AvansRobotics/flexibleassembly_ws.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

**Follow the guide in the Universal Robots ROS driver repo:**
The Universal_Robots_ROS_Driver is included in this git repo, further info here:
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver <br /> 
Use the calibration steps if this is the first use of your UR with ROS. 
