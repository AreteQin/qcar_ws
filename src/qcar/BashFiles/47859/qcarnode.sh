#!/bin/bash

ls -l /dev |grep ttyTHS2
sudo chmod 666 /dev/ttyTHS2
source /opt/ros/melodic/setup.bash
source ~/qcar_ws/devel/setup.bash
export ROS_IP=192.168.1.74
export ROS_MASTER_URI=http://192.168.1.12:11311
rosrun qcar qcarnode.py