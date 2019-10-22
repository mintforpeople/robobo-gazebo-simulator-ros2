# robobo-gazebo-simulator

Repository for Robobo robot simulation in the Gazebo environment (ROS 2 Dashing)

## Requirements

* Ubuntu 18.04.x
* Robot Operating System - Dashing
* Gazebo 9

## Install ROS 2 Dashing
```bash
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt-get update
sudo apt install ros-dashing-desktop
sudo apt-get install ros-dashing-gazebo-ros
sudo apt-get install ros-dashing-gazebo-plugins
```
## Create ROS Workspace
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
colcon build
```

## Install Robobo messages for ROS 2
	* Download archive from https://github.com/mintforpeople/robobo-ros2-msgs
	* Decompress archive in workspace


## Load workspace
`source ros2_ws/install/setup.bash`

## Compile workspace
```bash
cd ros2_ws
colcon build --symlink-install
```

## Launch the model
```bash
ros2 launch robobo_gazebo robobo.launch.py
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
* /\<modelName>\/move_wheels
* /\<modelName>\/reset_wheels
* /\<modelName>\/move_pan_tilt

\<modelName\> is robobo by default.

## Run test script
```bash
ros2 run robobo_gazebo robobo_validation.py
```

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