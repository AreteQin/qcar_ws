#!/bin/bash

ls -l /dev |grep ttyTHS2
sudo chmod 666 /dev/ttyTHS2
source /opt/ros/melodic/setup.bash
source /home/nvidia/qcar_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/nvidia/qcar_ws/python
roslaunch qcar controller.launch