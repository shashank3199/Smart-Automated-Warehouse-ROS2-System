# Saws Turtlebot4 Teleop Setup

This package is part of the Smart Automated Warehouse System (SAWS) and is designed to set up the teleoperation and localization functionalities for the TurtleBot4 robot. The package provides launch files and nodes to perform SLAM, save maps, and set up localization using joystick inputs.

## Index

- [Introduction](#introduction)
- [Index](#index)
- [User Guide](#user-guide)
  - [Usage](#usage)
    - [SLAM Mode](#slam-mode)
    - [Localization Mode](#localization-mode)
- [Package Explanation](#package-explanation)
  - [Launch Files](#launch-files)
  - [Source Files](#source-files)
- [References](#references)

## User Guide

### Usage

#### SLAM Mode

To perform SLAM and save the map in the maps folder, use the following command:

```bash
ros2 launch saws_turtlebot4_teleop_setup saws_turtlebot4_teleop_setup.launch.py slam:=true
```

In this mode, pressing the 'x' button on the joystick saves the current version of the map.

#### Localization Mode

To set up localization of the robot within a saved map, use the following command:

```bash
ros2 launch saws_turtlebot4_teleop_setup saws_turtlebot4_teleop_setup.launch.py localization:=true
```

In this mode, pressing the 'box' button on the joystick saves the current localized position in the config file. This allows for sampling points needed for patrolling by Nav2 in the Smart Automated Warehouse System.

## Package Explanation

### Launch Files

#### saws_turtlebot4_teleop_setup.launch.py

This launch file sets up the environment for teleoperation, SLAM, and localization based on the provided parameters.

**Arguments:**

- `slam`: Enable SLAM mode (default: false)
- `map`: Specify the map name if using SLAM or localization (default: wyman160.yaml)
- `localization`: Enable localization mode (default: false)

**Functionality:**

- Includes the `joy_teleop.launch.py` from `turtlebot4_bringup` unless localization is enabled.
- Includes the `slam.launch.py` from `turtlebot4_navigation` if SLAM is enabled.
- Includes the `localization.launch.py` from `turtlebot4_navigation` if localization is enabled.
- Spawns the `joystick_listener` node to handle joystick inputs for saving maps and positions.

### Source Files

#### joystick_listener.cpp

This node subscribes to joystick inputs and AMCL pose updates. It saves the map and the robot's current pose based on the joystick inputs.

**Subscriptions:**

- `joy` (sensor_msgs/msg/Joy): For joystick inputs.
- `amcl_pose` (geometry_msgs/msg/PoseWithCovarianceStamped): For current pose updates.

**Functionality:**

- Pressing the 'x' button saves the map.
- Pressing the 'box' button saves the current position to `positions.yaml`.

### CMakeLists.txt

Defines the build process for the package. Specifies dependencies and installation paths.

### package.xml

Defines the package metadata and dependencies.

### config/positions.yaml

Stores sampled positions for patrolling.

### maps/

Contains the map files used for localization.

### launch/

Contains the main launch file `saws_turtlebot4_teleop_setup.launch.py`.

## References

- [TurtleBot4 Navigation](https://www.turtlebot.com/turtlebot4/)
- [ROS2 Documentation](https://docs.ros.org/en/foxy/index.html)
