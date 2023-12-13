# Lightweight Gazebo/ROS-based Simulator for Unmanned Aerial Vehicles (UAVs)
This package implements a lightweight quadcopter unmanned aerial vehicles (UAVs) simulator including various static and dynamic based on Gazebo/ROS. It also includes an optional PX4-based quadcopter simulation wrapper.

## Installation Guide
build the package:
```
git clone https://github.com/Zhefan-Xu/uav_simulator.git

cd ~/catkin_ws
catkin_make
```

setup environment variable. Add following to your ```~/.bashrc```
```
source path/to/uav_simulator/gazeboSetup.bash
```

### Run
```
roslaunch uav_simulator start.launch
```

