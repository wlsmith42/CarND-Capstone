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
[image2]: ./imgs/sim.gif "sim"
---


## The Team

###Weston Smith

* Project Contributions: Team Lead, Traffic Light Detector, Traffic Light Classifier ROS Interface.

* Github: [wlsmith42](https://github.com/wlsmith42)

* Email: wlsmith42@students.tntech.edu

###Derza Arsad

* Project Contributions: Traffic Light Classifier

* Github: [derzaarsad](https://github.com/derzaarsad)

* Email: derza.arsad@yahoo.com


###Senlin Wang

* Project Contributions: Waypoint Updater

* Github: [wangsenlinautonomous](https://github.com/wangsenlinautonomous)

* Email: senlin.007@163.com

###Yumie Minakami

* Project Contributions: Waypoint Updater

* Github: [yminakami](https://github.com/yminakami)

* Email: mk66.yumie@gmail.com

###Yasser Abdallah

* Project Contributions: Drive-by-Wire

* Github: [yasserabdallah4](https://github.com/yasserabdallah4)

* Email: yasser.abdallah4@gmail.com


## Project Introduction

The following image shows the architecture for Carla, Udacity's self driving car. In this project, we were focused on implementing three ROS nodes across three vehicle subsystems: Traffic Light Detection, Waypoint Updater, and DBW. The tasks related to each vehicle subsystem are explained in more detail below.
![alt text][image1]

### Perception
The perception subsystem consists of the traffic light detector and traffic light classifier. The traffic light detector is the ROS node that receives camera images from the vehicle. Once an image is received it calculates if the vehicle is close to a traffic light using a list of stop line positions that correspond to the line where the car should stop for a given intersection. Images are processed at a rate of 1 image classified per 5 images received as long as they meet the distance requirement to a traffic light; this greatly reduces system overhead and allows for better results when running the project on the simulator. If the vehicle is approaching a traffic light, the image is then passed on to the classifier to determine the state of the light: Red, Yellow, Green, or Unknown. Once the image class is determined, the state of the light is then published to the ROS topic `upcoming_red_light_pub` at the same rate that camera images are published. For a more information about the traffic light detector, check out the code at `/ros/src/tl_detector/tl_detector.py`

We trained separate networks for the traffic light detection on the simulated data and the real data. The following data are used for the trainings:

1. For real data we used [dataset sdcnd capstone](https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/edit), we've got the data from our colleague [Michael Karg](https://github.com/micjey/CarND-Capstone)
2. For simulated data we used [Dataset from Alex Lechner](https://www.dropbox.com/s/vaniv8eqna89r20/alex-lechner-udacity-traffic-light-dataset.zip?dl=0), please check his repository on https://github.com/alex-lechner

We used SSD network which is recommended by [Object Detection Lab](https://github.com/udacity/CarND-Object-Detection-Lab) the version that we used is [SSD Inception V2 COCO 2017/11/17](http://download.tensorflow.org/models/object_detection/ssd_inception_v2_coco_2017_11_17.tar.gz).

At first we tried to do the normal way of detection and classification in which the outputs of the network are directly green, red, or yellow. However we noticed that the detection was very bad, even for detection on the training data itself. As a result we decided to do something else for the detection and classification:

1. Splitting the output into 3 classes as before means that the detector could be more prone to error caused by a bad training data distribution, therefore it is in our opinion better to focus on only detecting where the traffic light is and let another classifier detect the color of the traffic light. In this case the proportion of unique training data in a class is also increased a lot, because we can merge all the green, red and yellow data into a single traffic light class.

2. After we successfully identify the traffic light, we use a simple image processing to detect its color. First of all, the traffic images are cropped and resized to 32 by 32 pixels based on the detected position from the SSD network. Then the images are masked based on the hsv color and only the brightness channel is taken. After that, the image is splitted into 3 regions from top to the bottom (red, yellow, green). Some regions on the right and left sides are cropped to eliminate the background from the feature calculation. The calculated feature is the mean of brightness from each region where the region with the biggest mean is the region where the light is on. The followings are the image from HSV channels.

![traffic hsv](imgs/traffic_hsv.png)

The following is the code snippet from the feature extraction:

```
def create_feature(self,rgb_image):

        #Convert image to HSV color space
        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

        #Create and return a feature value and/or vector
        brightness_channel = hsv[:,:,2]
        rows = brightness_channel.shape[0]
        cols = brightness_channel.shape[1]
        mid = int(cols/2)

        red_region = brightness_channel[:int(rows/3),(mid-10):(mid+10)]
        yellow_region = brightness_channel[int(rows/3):int(2*rows/3),(mid-10):(mid+10)]
        green_region = brightness_channel[int(2*rows/3):,(mid-10):(mid+10)]

        feature = [0,0,0]
        feature[0] = np.mean(green_region)
        feature[1] = np.mean(red_region)
        feature[2] = np.mean(yellow_region)

        return feature
```

### Planning

The planning section contains waypoint loader and waypoint updater node. The waypoint loader is provided by Autoware, so in the project we only focus on waypoint updater part.

The basic idea of waypoint updater is to get position information from current_pose topic and get basic waypoints information from base_waypoint topic then publish final waypoint to final_waypoint topic accordingly. Also waypoint updater can consider traffic light and obstacle situations

So waypoint updater contains the following items:

* Get position information
* Get basic waypoint information
* Get traffic light information
  - Decelerate waypoints
* Publish final waypoint
  - KD tree introduction
  - Get the closest waypoint



<img src="https://user-images.githubusercontent.com/40875720/55613528-73845a80-57bd-11e9-8cf4-641de58c8f7f.PNG" width="400">

Now I'd like to go deeper to some subcomponents of waypoint updater

#### Get position information

Declare a subscriber to subscribe current_pose topic, then using a call back function to get positioning information.
For more information please see the attached code:

```
# Define a subscriber to subscribe current_pose to get positioning information
# Also declare a pose_cb call back function to do some post processing work after receiving the topic
rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

# Define a call back function to init pose variable when receiving current_pos topic
def pose_cb(self, msg):
        self.pose = msg
```

#### Get basic waypoint information

Declare a subscriber to subscribe base_waypoints topic, then using  a call back function to get base waypoint information.
Then transfer waypoints from 3D to 2D

```
from scipy.spatial import KDTree

rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

def waypoints_cb(self, waypoints):
        #Get the basic waypoints from waypoint loader.This action only need to be done once
        self.base_waypoints = waypoints

        #Got 2d waypoints from 3d waypoints
        if not self.waypoints_2d:
	     # Only take x and y in to consideration to get 2D waypoints
             self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
             # Build KD tree by using KDTree function, KDTree function scipy.spatial lib
	     self.waypoint_tree = KDTree(self.waypoints_2d)
```

#### Get traffic light information

Declare a subscriber to subscribe traffic_waypoint topic, then using a call back function to get the index of stop line. Well prepared for deceleration waypoint topic.

```
rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

```

##### Deceleration waypoints

The purpose of deceleration waypoints function is to decelerate the vehicle until stop if there is a red light in front of the vehicle.
For this purpose we only need to update the vehicle speed(twist.linear) of the each waypoints.
In the case, we will create a new waypoint list to update the vehicle speed, otherwise it will over write the original vehicle speed, then the behavior of the vehicle can be strange in the following loops.

Also to get a smooth vehicle speed is quite important. we use `vel = math.sqrt(2 * MAX_DECEL * SAFETY_FACTOR * dist)` to calculate the velocity according to dist. The result is showing in the following pic:

<img src="https://user-images.githubusercontent.com/40875720/55624827-a4728880-57d9-11e9-911e-08ea61ea5bd4.PNG" width="400">

```
def decelerate_waypoints(self, waypoints, closest_idx):
        result = []
        for i, wp in enumerate(waypoints):
            new_point = Waypoint()
            new_point.pose = wp.pose

	    # Should stop in front of the stop line, otherwise it will be dangerous to break law
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)  

            # the car stops at the line
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * SAFETY_FACTOR * dist)
            if vel < 1.0:
                vel = 0.0

            new_point.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            result.append(new_point)

        return result
```


#### Publish final waypoints

##### KD Tree introduction

In computer science, a k-d tree (short for k-dimensional tree) is a space-partitioning data structure for organizing points in a k-dimensional space. k-d trees are a useful data structure for several applications, such as searches involving a multidimensional search key (e.g. range searches and nearest neighbor searches). k-d trees are a special case of binary space partitioning trees.

Basically, it contains two parts: Build and Search. In our case, I use `self.waypoint_tree = KDTree(self.waypoints_2d)` function to build KD tree, use `waypoint_tree.query([x, y], 1)[1]` function to search.

Find more information in the following links:

[https://baike.baidu.com/item/kd-tree/2302515?fr=aladdin](https://baike.baidu.com/item/kd-tree/2302515?fr=aladdin)

[https://en.wikipedia.org/wiki/K-d_tree](https://en.wikipedia.org/wiki/K-d_tree)

##### Get the closest waypoints

The purpose of this section is to find the closest waypoint **in front of** the vehicle. Basically this function can find the index of the closest waypoint, then be well prepared to publish the waypoints to the final_waypoints topic.

For more details please refer to the code below,I made some comments to the code

```
def get_closest_waypoint_idx(self):
        # Get the coordinates of our car
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        # Get the index of the closest point
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0: # Our car is in front of the closest waypoint
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx
```

The result is shown below:

<img src="https://user-images.githubusercontent.com/40875720/55612987-22279b80-57bc-11e9-83e4-91e97e06c8ec.PNG" width="400">

### Control

The Drive-by-wire is implemented by to two modules; dbw_node.py and twist_controller.py. In dbw_node.py, we need to handle ROS subscribers for the /current_velocity, /twist_cmd, and /vehicle/dbw_enabled topics and publish throttle, steering and brake signals.

The twist controller (including the imported yaw controller) manages to set the desired linear and angular velocity with the help of a PID controller which outputs the necessary actuator signals. We subscribe to the desired linear and angular velocity via the twist_cmd topic which is published by the Waypoint Follower Node.

For the Break if the desired speed is less than 1 mph, 40% of the maximum break is applied, else if the actual car speed is less than 1 mph maximum break is applied


## Results
When all three vehicle subsystems are combined, the car was able to autonomously drive through the test track, stopping at red lights as needed. The results can be seen in the GIF below:

![alt text][image2]

## Limitations
Overall, the project works very well with the only difficulties being related to development & testing in Udacity's provided project workspace. The main problems we faced were:

* The project workspace does not have a lot of processing power, and this can result in severe simulator lag despite countless code optimizations which can cause the vehicle to veer from the waypoints.
* Testing the bag file for the real world test was basically impossible in the workspace due to insufficient space to load the file.

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
./run.sh
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
