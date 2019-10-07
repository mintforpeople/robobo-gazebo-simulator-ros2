# robobo-gazebo-simulator

Repository for Robobo robot simulation in the Gazebo environment.

## Requirements

* Ubuntu 16.04
* Robot Operating System - Kinetic
* Gazebo7

## Installation

This model uses the original Robobo ROS messages, so it is necessary to use the robobo_msgs package, avaliable on https://github.com/mintforpeople/robobo-ros-msgs/tree/master/robobo_msgs.
Clone the respository in your own workspace and compile:


```bash
$ cd <catkin_ws>/src
$ git clone https://github.com/mintforpeople/robobo-gazebo-simulator
$ git clone https://github.com/mintforpeople/robobo-ros-msgs/tree/master/robobo_msgs
$ catkin_make
```

## Basic Usage

You must do the first two steps in each new terminal you need to use the model:

```bash
$ cd <catkin_ws>/src
$ source devel/setup.bash
```

To launch the model:

```bash
$ roslaunch robobo_gazebo robobo.launch
```

To interact with the model you have the following ROS topics and services. They are the same ones used in the real Robobo, there is more information here: https://github.com/mintforpeople/robobo-programming/wiki/ROS.

Topics availables:
* /\<modelName\>/accel
* /\<modelName\>/camera/camera_info
* /\<modelName\>/camera/image/compressed
* /\<modelName\>/irs
* /\<modelName\>/orientation
* /\<modelName\>/pan
* /\<modelName\>/tilt
* /\<modelName\>/wheels

Services availables:
* /\<modelName>\/movewheels
* /\<modelName>\/resetWheels
* /\<modelName>\/movePanTilt

\<modelName\> is robot by default but it can be changed for other name in the launch file.

## Remark
This package includes one node in python with the function of publishing all infrared sensor values in only one topic. This program reads all topics published by plugin infrared_range.cpp in each ray sensor of the model and brings them all together in one topic, like in the real Robobo.

## License

robobo-gazebo-simulator is available under the Apache 2.0 license. See the LICENSE file for more info.

## Acknowledgement
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 



