[![ROS2 Tag](https://img.shields.io/badge/ROS2-Humble-orange.svg)](https://docs.ros.org/en/humble/index.html)

# Smart Automated Warehouse System (SAWS) Project

Welcome to the Smart Automated Warehouse System (SAWS) project repository. This repository contains all the necessary packages and configurations to set up and operate an intelligent warehouse system using TurtleBot4 and UR5 robots. The system integrates functionalities such as teleoperation, localization, SLAM, gripper control, and navigation.

## Team Members

-   Shashank Goyal: sgoyal18@jhu.edu
-   Tarun Prasad Senthilvel: [tsenthi1@jhu.edu](mailto:tsenthi1@jhu.edu)

## Table of Contents

-   [Introduction](#introduction)
-   [Setup Instructions](#setup-instructions)
    -   [Creating a ROS Workspace](#creating-a-ros-workspace)
    -   [Cloning the Repository](#cloning-the-repository)
    -   [Building the Workspace](#building-the-workspace)
    -   [Sourcing the Workspace](#sourcing-the-workspace)
-   [Package List](#package-list)
-   [References](#references)

## Introduction

The SAWS project aims to design and implement an intelligent system for inventory management using TurtleBot4 and UR5 robots. The system leverages ROS2 for robot control, Computer Vision, Robot Motion Planning, and Robot System Programming.

## Setup Instructions

### Creating a ROS Workspace

To start, create a ROS workspace on your TurtleBot4:

```bash
mkdir -p ~/saws_workspace/src
cd ~/saws_workspace/src
```

### Cloning the Repository

Clone this repository into the `src` folder of your ROS workspace:

```bash
cd ~/saws_workspace/src
git clone https://github.com/jhu-rsp/Smart-Automated-Warehouse-System.git
```

### Building the Workspace

After cloning the repository, build your workspace:

```bash
cd ~/saws_workspace
colcon build
```

### Sourcing the Workspace

Source the setup script to overlay the workspace on your ROS environment:

```bash
source ~/saws_workspace/install/setup.bash
```

## Package List

This repository contains the following packages, each serving a specific role in the SAWS project:

-   **saws_bringup**: Launch and bringup files for the SAWS project.
-   **saws_description**: URDF and mesh files for robot descriptions.
-   **saws_handeye_calibration**: Tools and scripts for hand-eye calibration.
-   **saws_gripper_actionlib**: Action libraries for gripper control.
-   **saws_gripper_msgs**: Custom messages for gripper control.
-   **saws_teleop_joy**: Joystick teleoperation for the robots.
-   **saws_turtlebot4_nav2_bringup**: Navigation2 bringup files for TurtleBot4.
-   **saws_turtlebot4_nav2_move_to**: Scripts and nodes for moving TurtleBot4 to specific points.
-   **saws_turtlebot4_teleop_setup**: Setup for teleoperation and localization of TurtleBot4.
-   **saws_ur5_control_loop**: Control loops for the UR5 robot.
-   **saws_ur5_joint_interpolator**: Joint interpolator for smooth UR5 movements.
-   **saws_ur5_pickup**: Nodes and scripts for pickup operations with UR5.
-   **saws_ur5_rrmc**: Robot motion control for the UR5 robot.

## References

For more detailed information on ROS and TurtleBot4, refer to the following resources:

-   [ROS2 Documentation](https://docs.ros.org/en/foxy/index.html)
-   [TurtleBot4 Documentation](https://www.turtlebot.com/turtlebot4/)
-   [Docker Documentation](https://docs.docker.com/)
