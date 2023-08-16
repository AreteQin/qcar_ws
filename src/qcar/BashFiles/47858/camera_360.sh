#!/bin/bash

ls -l /dev |grep ttyTHS2
sudo chmod 666 /dev/ttyTHS2
export PYTHONPATH=”/home/nvidia/qcar_ws/python”
source /opt/ros/melodic/setup.bash
source ~/qcar_ws/devel/setup.bash
export ROS_IP=192.168.1.95
export ROS_MASTER_URI=http://192.168.1.12:11311
sudo chmod +x /home/nvidia/qcar_ws/src/qcar/src/csinode.py
rosrun qcar csinode.py