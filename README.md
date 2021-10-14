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

**Follow the guide in the Universal Robots ROS driver repo:**
https://github.com/UniversalRobots/Universal_Robots_ROS_Driver <br /> 
Use the calibration steps if this is the first use of your UR with ROS

**Install this repository from Source:**

First set up a catkin workspace (see [this tutorials](http://wiki.ros.org/catkin/Tutorials)).  
Then clone the repository into the src/ folder. It should look like /path/to/your/catkin_workspace/src/FlexibleAssembly.  
Make sure to source the correct setup file according to your workspace hierarchy, then use catkin_make to compile.  

Assuming your catkin workspace folder is ~/catkin_ws, you should use the following commands:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/AvansRobotics/FlexibleAssembly.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```
