#!/bin/bash

ls -l /dev |grep ttyTHS2
sudo chmod 666 /dev/ttyTHS2
source /opt/ros/melodic/setup.bash
source /media/nvidia/Sony32/qcar_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/media/nvidia/Sony32/qcar_ws/python
roslaunch qcar controller.launch