#!/bin/bash

ls -l /dev |grep ttyTHS2
sudo chmod 666 /dev/ttyTHS2
source /opt/ros/melodic/setup.bash
source /media/nvidia/Sony32/qcar_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/nvidia/qcar_ws/python
export ROS_IP=192.168.1.75
export ROS_MASTER_URI=http://192.168.1.12:11311
roslaunch qcar d435_and_imu.launch