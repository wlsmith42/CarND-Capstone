#!/bin/bash
#script to build catkin workspace and run ROS application



#change directory to ROS folder - location of the catkin workspace
cd ros

#build the workspace
catkin_make

#add environment variables to the path
source devel/setup.bash

#Run the ROS application
roslaunch launch/styx.launch
