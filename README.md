# Lightweight Gazebo/ROS-based Simulator for Unmanned Aerial Vehicles (UAVs)
This package implements a lightweight quadcopter unmanned aerial vehicles (UAVs) simulator including various static and dynamic based on Gazebo/ROS. It also includes an optional PX4-based quadcopter simulation wrapper.

**Author**: [Zhefan Xu](https://zhefanxu.com/) from the Computational Engineering & Robotics Lab (CERLAB) at Carnegie Mellon University (CMU).

## I. Installation Guide
This repo has been tested on [ROS Melodic](http://wiki.ros.org/ROS/Installation) with Ubuntu 18.04 and [ROS Noetic](http://wiki.ros.org/ROS/Installation) with Ubuntu 20.04.
#### a. Non-PX4 Simulator (Required)
To install the non-px4 simulator, please follow the standard catkin package make process as follows:
```
git clone https://github.com/Zhefan-Xu/uav_simulator.git

cd ~/catkin_ws
catkin_make
```

setup environment variable. Add following to your ```~/.bashrc```
```
source path/to/uav_simulator/gazeboSetup.bash
```
#### b. PX4-based Simulator Wrapper (Optional but Recommended)
Please make sure that you have follow the previous steps to build the non-px4 simulator.

**Step 1**: Install vehicle models and make it compatible with your current ROS. The following lines give the summaries:

&#x1F34E; Current PX4 version has some problems with offboard mode, please use v1.12.0 as modified in the following lines:&#x1F34E;
```
cd directory/to/install # this should not be your catkin workspace

git clone https://github.com/PX4/PX4-Autopilot.git --recursive --branch v1.12.0
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh # this step will ask you to reboot

# Please make sure you reboot after the previous step
cd /path/to/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
```
**Step 2**: Add the following script to ```~/.bashrc```. Remember to replace ```<PX4-Autopilot_clone>``` with the path to your PX4-Autopolot directory. This step will you setup the environment variables properly.
```
source <PX4-Autopilot_clone>/Tools/setup_gazebo.bash <PX4-Autopilot_clone> <PX4-Autopilot_clone>/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:<PX4-Autopilot_clone>
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:<PX4-Autopilot_clone>/Tools/sitl_gazebo
```
**Step 3**: Install [MAVROS](https://docs.px4.io/master/en/ros/mavros_installation.html) for communication.
```
sudo apt-get install ros-[melodic/noetic]-mavros ros-[melodic/noetic]-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh  
```



## II. Quick Start
a. To launch the non-PX4 simulator with a quadcopter:
```
roslaunch uav_simulator start.launch
```

You should be able to see a customized quadcopter in a predefined gazebo environment as shown below: 

![simulator](https://github.com/Zhefan-Xu/uav_simulator/assets/55560905/32f1a2d1-becb-4854-b6e1-161118b319f4)


b. To launch the PX4 simulator with a quadcopter:
```
roslaunch uav_simulator px4_start.launch
```

You should be able to see a PX4 IRIS quadcopter in a predefined gazebo environment as shown below: 

![px4_simulator](https://github.com/Zhefan-Xu/uav_simulator/assets/55560905/fbcb0100-51cf-445a-bfa0-25dc96ab022e)

## III. Keyboard Control
Our non-px4 customized simulator supports the keyboard control function. You are able to control the quadcopter motion **when you click the keyboard controller panel** shown as below:

![keboard_control](https://github.com/Zhefan-Xu/uav_simulator/assets/55560905/989fd8eb-28d6-4927-a021-2b191765ed82)


## IV. Simulation Environments
- There are various predefined environments in this package and you can easily switch environment when you modify the launch file located in ```uav_simululator/launch/start.launch``` or ```uav_simululator/launch/px4_start.launch```. All the predifined environments are listed in the lanuch files.
- There are some environments which contains dynamic objects (e.g. moving persons) and we call those dynamic environments. You can distinguish those dynamic environments by the environments' names. For example, the environment name ```floorplan1_dynamic_16.world``` indicates that there are 16 dynamic objects in the floorplan1 environment.


## V. ROS Topics



