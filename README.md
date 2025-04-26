# qcar

## Dependencies
```bash
sudo apt install ros-melodic-hector-geotiff
```

## Usage
```bash
# To initiate the QCar and publish the front camera images
sudo bash src/qcar/BashFiles/47858/control_with_front_camera.sh
# To send the command from the keyboard
rosrun qcar command
# or
/home/qin/qcar_ws/src/qcar/cmake-build-debug/devel/lib/qcar/command

# To control the QCar only 
sudo bash /home/nvidia/qcar_ws/src/qcar/BashFiles/keyboard_local.sh

# To publish the cameras images only
rosrun qcar csi
```
