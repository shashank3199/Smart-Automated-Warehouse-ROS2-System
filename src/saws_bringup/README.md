# SAWS Bringup Package

This repository contains the bringup files for the Smart Automated Warehouse System (SAWS) project. The `saws_bringup` package is designed to initialize and configure the necessary nodes and settings for the SAWS project.

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
  - [launch/saws_bringup.launch.py](#launchsaws_bringuplaunchpy)
  - [launch/ur5_bringup.launch.py](#launchur5_bringuplaunchpy)
  - [package.xml](#package.xml)
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
colcon build --packages-select saws_bringup
source install/setup.bash
```

### Running the Launch Files

To launch the SAWS bringup process, execute the respective launch file using the ROS 2 launch system:

```
ros2 launch saws_bringup saws_bringup.launch.py
```

For bringing up the UR5 robot and related nodes, execute:

```
ros2 launch saws_bringup ur5_bringup.launch.py
```

## Directory Structure

```
saws_bringup/
├── CMakeLists.txt
├── launch/
│   ├── saws_bringup.launch.py
│   └── ur5_bringup.launch.py
└── package.xml
```

## File Roles

### CMakeLists.txt

**Purpose:**

- Sets up the build configuration for the `saws_bringup` package.
- Defines the project and compiler options.
- Specifies the installation of the `launch` directory to the share folder.

### launch/saws_bringup.launch.py

**Purpose:**

- Launches the `saws_pickup` and `saws_control_loop` nodes.
- Includes the launch description for the `saws_ur5_pickup` package.
- Defines the `saws_control_loop` node, which manages the control loop for the UR5 robot.

**Usage:**

- To launch the SAWS bringup process, execute this launch file using the ROS 2 launch system:
  ```bash
  ros2 launch saws_bringup saws_bringup.launch.py
  ```

### launch/ur5_bringup.launch.py

**Purpose:**

- Initializes and launches the UR robot driver and other necessary nodes for the UR5 robot.
- Includes the launch description for the `ur_robot_driver` package with specific arguments for the UR5 robot.
- Defines the `ur5_joint_interpolator` node for interpolating joint positions of the UR5 robot.

**Usage:**

- To bring up the UR5 robot and related nodes, execute this launch file using the ROS 2 launch system:
  ```bash
  ros2 launch saws_bringup ur5_bringup.launch.py
  ```

### package.xml

**Purpose:**

- Defines the package metadata including the name, version, description, maintainer, and license.
- Specifies the build and test dependencies required by the package.

## Summary

The `saws_bringup` package is essential for setting up and initializing the nodes required for the Smart Automated Warehouse System. By organizing the launch files and specifying the necessary configurations, it simplifies the process of bringing up the UR5 robot and related control systems. The provided launch files can be executed to start the various components of the SAWS project, ensuring a smooth and efficient bringup process.
