# UR5_Vision_Assemble
Use ROS2 and Gazebo to simulate UR5e RObot with the robotiq epick gripper and realsense camera to perform a vision pick and place task.

The code base is mainly from [UR5e_Vision_Assemble](https://github.com/zitongbai/UR5e_Vision_Assemble). 

The other part like vision grasp is from [ur5_ROS-Gazebo](https://github.com/lihuang3/ur5_ROS-Gazebo)

And the urdf of the epick gripper is from [ros2_epick_gripper](https://github.com/PickNikRobotics/ros2_epick_gripper)

The moveit py interface is implemented using [pymoveit2](https://github.com/AndrejOrsula/pymoveit2), because moveit2 hasn't integrated the python interface in ROS2 humble, if we want to use the official moveit python interface in humble, we muss build the moveit from source, which is too large and time cost.

## Reasons for choosing the repositories

- [UR5e_Vision_Assemble](https://github.com/zitongbai/UR5e_Vision_Assemble) has achieved two robot arm in gazebo, so it saves so much time for setup the whole simulation environment. And from the other project I submit(ur5e_suction_pybullet) I have achieved the task using pybullet, which is a simple simulator. So I adapt the idea to gazebo for more realistic usage.

- [ur5_ROS-Gazebo](https://github.com/lihuang3/ur5_ROS-Gazebo) has finished a automatically vision-based pick and place task, which is development in ROS1. So I adapt the code for vision-based pick and place



# Environment

* Ubuntu 22.04
* ROS2 humble

# Installation

Before installation, you need some dependencies installed.

## Gazebo classic

Unlike ROS, Gazebo is not installed when you install ROS2. You need to install it manually.

```bash
sudo apt update
sudo apt install gazebo
```

## Moveit2

```bash
sudo apt install ros-humble-moveit*
```

## ros2 control
```bash
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
```
## dependencies of image

```bash
sudo apt install ros-humble-image-pipeline
sudo apt install ros-humble-compressed-image-transport
sudo apt install ros-humble-compressed-depth-image-transport
sudo apt install ros-humble-vision-msgs
pip install opencv-python torch numpy
```

## dependencies of gazebo

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-realsense2-description
sudo apt install ros-humble-gazebo-ros2-control
```

## Build

Build the repo
```bash
cd ~/ur5e_suction_gazebo
colcon build --symlink-install
```

# Usage

Start the simulation environment
```bash
cd ~/ur5e_suction_gazebo
source install/setup.bash
ros2 launch suction_ur5e_gripper_moveit_config suction_ur5e_gripper_sim_moveit.launch.py # To start the gazebo and also the moveit
ros2 launch suction_ur5e_gripper_control vision.launch.py # To start the vision node
```

Or directly start both
```bash
cd ~/ur5e_suction_gazebo
source install/setup.bash
ros2 launch bringup simulation.launch.py
```


Launch the demo to do pickup and place task
```bash
# in another terminal
cd ~/ur5e_suction_gazebo
source install/setup.bash
ros2 run ur5e_gripper_control ur5_move_arm.py
```


# Some notes

## Before usage

make sure you have all the dependences installed

make sure you have local gazebo model in `~/.gazebo/models`

## Something need to be cared about

1. The commit in xml file can not contain symbol ":"
2. Simulation need to set `use_sim_time=true`，better set in launch

# TODOs

1. Vision control now is just a demo to pickup a determined target object using a finetune position, should use the uf transform and also the depth image in realsense2 camera for intelligent vision position tracking
2. The epick vision mesh file for now is not loaded(use a simple cylinder instead), because it will stuck the gazebo if I use the original mesh file.


# Acknowledgment

* Gazebo grasp plugin for ROS2 [gazebo_pkgs](https://github.com/kongoncharuk/gazebo-pkgs)
* [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
* [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
* [ros2_epick_gripper](https://github.com/PickNikRobotics/ros2_epick_gripper)
