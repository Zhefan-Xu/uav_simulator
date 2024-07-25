# Lightweight Gazebo/ROS-based Simulator for Unmanned Aerial Vehicles (UAVs)
This package implements a lightweight quadcopter unmanned aerial vehicles (UAVs) simulator including various static and dynamic based on Gazebo/ROS. It also includes an optional PX4-based quadcopter simulation wrapper.

**Author**: [Zhefan Xu](https://zhefanxu.com/), Computational Engineering & Robotics Lab (CERLAB) at Carnegie Mellon University (CMU).

If you find this work helpful, kindly show your support by giving us a free ⭐️. Your recognition is truly valued.

This repo can be used as a standalone package and also comes as a module of our [autonomy framework](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy).

## I. Installation Guide
This repo has been tested on [ROS Melodic](http://wiki.ros.org/ROS/Installation) with Ubuntu 18.04 and [ROS Noetic](http://wiki.ros.org/ROS/Installation) with Ubuntu 20.04.
#### a. Non-PX4 Simulator (Required)
To install the non-PX4 simulator, please follow the standard catkin package make process as follows:
```
sudo apt-get install ros-[melodic/noetic]-mavros* # this package depends on mavros related ROS packages
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
**Step 3**: Install geographiclib datasets for PX4 simulation.
```
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
Our non-PX4 customized simulator supports the keyboard control function. You are able to control the quadcopter motion **when you click the keyboard controller panel** shown as below:

![keboard_control](https://github.com/Zhefan-Xu/uav_simulator/assets/55560905/989fd8eb-28d6-4927-a021-2b191765ed82)


## IV. Simulation Environments
- There are various predefined environments in this package and you can easily switch environment when you modify the launch file located in ```uav_simululator/launch/start.launch``` or ```uav_simululator/launch/px4_start.launch```. All the predifined environments are listed in the lanuch files.
- There are some environments which contains dynamic objects (e.g. moving persons). You can distinguish those dynamic environments by the environments' names. For example, the environment name ```floorplan1_dynamic_16.world``` indicates that there are 16 dynamic objects in the floorplan1 environment.

One example of the dynamic environment is shown as below:

![Screenshot from 2023-12-18 20-36-34](https://github.com/Zhefan-Xu/uav_simulator/assets/55560905/4792edb1-1920-4097-b5aa-b8be64fe7b4e)


## V. ROS Topics
Here lists some important ROS topics related to the simulator:
- **Non-PX4 Simulator:**
  - ```/CERLAB/quadcopter/cmd_acc```: The command acceleration to the quadcopter.
  - ```/CERLAB/quadcopter/pose```: The ground truth pose of the quadcopter.
  - ```/CERLAB/quadcopter/odom```: The ground truth odom of the quadcopter.
  - ```/CERLAB/quadcopter/setpoint_pose```: The command pose to the quadcopter.
  - ```/camera/color/image_raw```: The color image from the onboard camera.
  - ```/camera/depth/image_raw```: The depth image from the onboard camera.
  - ```/camera/depth/points```: The depth cloud from the onboard camera.
- **PX4 Simulator**
  - ```/mavros/setpoint_raw/attitude```: The command to the quadcopter.
  - ```/mavros/local_position/pose```: The ground truth pose of the quadcopter.
  - ```/mavros/local_position/odom```: The ground truth odom of the quadcopter.
  - ```/mavros/setpoint_position/local```: The command pose to the quadcopter.
  - ```/camera/color/image_raw```: The color image from the onboard camera.
  - ```/camera/depth/image_raw```: The depth image from the onboard camera.
  - ```/camera/depth/points```: The depth cloud from the onboard camera.

## VI. Random World Generation
The simulator supports random (dynamic) world generation using the following commands:
```
# Go to the generator directory
cd uav_simulator/scripts

# Run random world generator
python3 generate_random_world.py 
```
The parameter configuration file is in ```uav_simulator/scripts/world_generator.yaml``` 


By default, the generated world will be saved under ```uav_simulator/worlds/generated_env/generated_env.world```. Please add this to ```start.launch``` file to launch it for experiments.
The example generated environment is visualized:


https://github.com/user-attachments/assets/b6f2c4a0-5868-4f07-ab32-48e59bbc814f



The generated world also support map generation for map_manager and dynamic obstacle detection by onboard_detector. The map pcd files can be generated by turning on the ```generate_map: True``` parameter. 
The example of the generated environment with the generated map and dynamic obstacles are shown below:



https://github.com/user-attachments/assets/22b5e256-2eb1-423c-b000-9d54690603b1


