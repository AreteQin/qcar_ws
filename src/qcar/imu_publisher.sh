#!/bin/bash

source /opt/ros/melodic/setup.bash
source ~/qcar_ws/devel/setup.bash
export ROS_IP=100.85.255.99
export ROS_MASTER_URI=http://100.86.255.112:11311
chmod +x /home/nvidia/qcar_ws/src/qcar/src/IMU_publisher.py
rosrun qcar IMU_publisher.py