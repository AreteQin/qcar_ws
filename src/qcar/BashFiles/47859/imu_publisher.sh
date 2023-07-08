#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/qcar_ws/devel/setup.bash
export ROS_IP=192.168.1.74
export ROS_MASTER_URI=http://192.168.1.12:11311
chmod +x /home/nvidia/qcar_ws/src/qcar/src/IMU_publisher.py
rosrun qcar IMU_publisher.py