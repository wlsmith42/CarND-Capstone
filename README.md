# Programming an Autonomous Vehicle
Self-Driving Car Engineer Nanodegree Program

---
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The goals / steps of this project are to write ROS nodes to implement the core functionality of an autonomous vehicle system including:

* Perception through Traffic Light Detection
* Vehicle Control using Drive-by-Wire Systems
* Path Planning by Updating and Following Waypoints

[//]: # (Image References)

[image1]: ./imgs/ros_architecture.png "ros_arch"

---

 
## The Team

* [Weston Smith](weston.smith42@gmail.com) - Team Lead, Traffic Light Detector, Traffic Light Classifier ROS Interface.
* [Derza Arsad](derza.arsad@yahoo.com) - [Traffic Light Classifier]
* [Senlin Wang](senlin.007@163.com) - [Waypoint Updater]
* [Yumie Minakami](mk66.yumie@gmail.com) - [Waypoint Updater]
* [Yasser Abdallah](yasser.abdallah4@gmail.com) - [Drive by Wire]
* [Michael Karg](micjey@gmx.de) - [Drive by Wire]


## Project Introduction

The following image shows the architecture for Carla, Udacity's self driving car. In this project, we were focused on implementing three ROS nodes across three vehicle subsystems: Traffic Light Detection, Waypoint Updater, and DBW. The tasks related to each vehicle subsystem are explained in more detail below.
![alt text][image1]

### Perception
The perception subsystem consists of the traffic light detector and traffic light classifier. The traffic light detector is the ROS node that receives camera images from the vehicle. Once an image is received it calculates if the vehicle is close to a traffic light using a list of stop line positions that correspond to the line where the car should stop for a given intersection. Images are processes at a rate of 1 image classified per 5 images received as long as they meet the distance requirement to a traffic light; this greatly reduces system overhead and allows for better results when running the project on the simulator. If the vehicle is approaching a traffic light, the image is then passed on to the classifier to determine the state of the light: Red, Yellow, Green, or Unknown. Once the image class is determined, the state of the light is then published to the ROS topic `upcoming_red_light_pub` at the same rate that camera images are published. For a more information about the traffic light detector, check out the code at `/ros/src/tl_detector/tl_detector.py`

[TODO: Classifier]

### Planning

[TODO: Waypoint Updater]

### Control

[TODO: DBW]

## Results

[TODO: Results]

## Basic Build Instructions

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
