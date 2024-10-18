#!/bin/bash

ls -l /dev |grep ttyTHS2
sudo chmod 666 /dev/ttyTHS2
source /opt/ros/melodic/setup.bash
source /home/nvidia/qcar_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/nvidia/qcar_ws/python
chmod +x /home/nvidia/qcar_ws/src/qcar/src/qcarnode.py
roslaunch qcar controller.launch