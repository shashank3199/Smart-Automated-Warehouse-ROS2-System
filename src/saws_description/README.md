# SAWS Description Package

This repository contains the description files for the Smart Automated Warehouse System (SAWS) project. The `saws_description` package provides the necessary URDF, configuration, and launch files for visualizing and simulating the SAWS project components in Gazebo.

## Index

- [Introduction](#introduction)
- [Index](#index)
- [User Guide](#user-guide)
  - [Downloading the Package](#downloading-the-package)
  - [Building the Package](#building-the-package)
  - [Running the Launch Files](#running-the-launch-files)
- [Directory Structure](#directory-structure)
- [File Roles](#file-roles)
  - [CMakeLists.txt](#cmakelists.txt)
  - [config/](#config)
    - [initial_positions.yaml](#initial_positions.yaml)
    - [ur_controllers.yaml](#ur_controllers.yaml)
  - [launch/](#launch)
    - [ur5_description.launch.py](#ur5_description.launch.py)
  - [package.xml](#package.xml)
  - [urdf/](#urdf)
    - [saws.urdf.xacro](#saws.urdf.xacro)
  - [worlds/](#worlds)
    - [warehouse.world](#warehouse.world)
- [Summary](#summary)

## User Guide

### Downloading the Package

To download the package, clone the repository using the following command:

```
git clone https://github.com/jhu-rsp/Smart-Automated-Warehouse-System.git
```

### Building the Package

Navigate to the root directory of the package and run the following commands:

```
colcon build --packages-select saws_description
source install/setup.bash
```

### Running the Launch Files

To launch the UR5 robot description and simulation in Gazebo, execute the following launch file using the ROS 2 launch system:

```
ros2 launch saws_description ur5_description.launch.py
```

## Directory Structure

```
saws_description/
├── CMakeLists.txt
├── config/
│   ├── initial_positions.yaml
│   └── ur_controllers.yaml
├── launch/
│   ├── ur5_description.launch.py
├── package.xml
├── urdf/
│   ├── saws.urdf.xacro
└── worlds/
    ├── warehouse.world
```

## File Roles

### CMakeLists.txt

**Purpose:**

- Sets up the build configuration for the `saws_description` package.
- Finds and includes necessary dependencies such as `ament_cmake`, `ur_description`, `realsense2_description`, and `turtlebot4_description`.
- Installs the `config`, `launch`, `urdf`, and `worlds` directories to the share folder.

### config/

This directory contains configuration files for initializing the robot's joints and controllers.

#### initial_positions.yaml

**Purpose:**

- Specifies the initial joint positions for the UR5 robot.

**Usage:**

- Sets the initial positions of the UR5 robot joints during simulation.

#### ur_controllers.yaml

**Purpose:**

- Configures the controller settings for the UR5 robot, including the joint state broadcaster and the position controller.

**Usage:**

- Configures the controllers that manage the UR5 robot's joints during simulation.

### launch/

This directory contains the launch files that initialize and manage the nodes required for visualizing and simulating the SAWS project components.

#### ur5_description.launch.py

**Purpose:**

- Launches the UR5 robot description in Gazebo along with necessary nodes and parameters.
- Sets environment variables and includes the necessary launch descriptions for `ros_gz_sim`.
- Spawns the URDF model of the SAWS project in Gazebo and starts the robot state publisher.

**Usage:**

- To launch the description of UR5 robot mounted with the Realsense Camera and the Robotiq Gripper and simulation in Gazebo, execute this launch file using the ROS 2 launch system:
  ```bash
  ros2 launch saws_description ur5_description.launch.py
  ```

### package.xml

**Purpose:**

- Defines the package metadata including the name, version, description, maintainer, and license.
- Specifies the build and runtime dependencies required by the package.

### urdf/

This directory contains the URDF files that describe the robot model.

#### saws.urdf.xacro

**Purpose:**

- Defines the URDF model for the SAWS project, including the UR5 robot, RealSense camera, and Robotiq gripper.
- Includes necessary parameters and macros for configuring the robot model.
- Generates the complete URDF model of the SAWS project for simulation in Gazebo.

### worlds/

This directory contains the world files for Gazebo simulation.

#### warehouse.world

**Purpose:**

- Defines the Gazebo world environment for the SAWS project, including the warehouse layout and various models such as shelves, barriers, and people.
- Sets up the simulation environment in Gazebo where the SAWS project components will operate.

## Summary

The `saws_description` package is essential for providing the necessary URDF, configuration, and launch files to visualize and simulate the SAWS project components in Gazebo. By organizing the configuration and launch files, it simplifies the process of bringing up the UR5 robot, RealSense camera, and Robotiq gripper in a simulated warehouse environment. The provided files can be executed to start the various components of the SAWS project, ensuring a smooth and efficient simulation setup.
