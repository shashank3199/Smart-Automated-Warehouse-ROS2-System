# SAWS TurtleBot4 Navigate to Goal

## Introduction

This package is part of the Smart Automated Warehouse System (SAWS) and provides a ROS2 node to navigate the TurtleBot4 to a specified goal position using the TurtleBot4Navigator. The node sets the initial pose and navigates the robot to the goal pose.

## Index

- [Introduction](#introduction)
- [Index](#index)
- [User Guide](#user-guide)
  - [Usage](#usage)
- [Package Explanation](#package-explanation)
  - [Launch Files](#launch-files)
  - [Source Files](#source-files)
- [References](#references)

## User Guide

### Usage

To navigate the TurtleBot4 from an initial pose to a goal pose, use the following command:

```sh
ros2 launch saws_turtlebot4_nav2_move_to navigate_to_goal.launch.py initial_x:=0.0 initial_y:=0.0 initial_w:=1.0 goal_x:=1.0 goal_y:=1.0 goal_w:=1.0
```

Replace the values of `initial_x`, `initial_y`, `initial_w`, `goal_x`, `goal_y`, and `goal_w` with the desired coordinates and orientation.

## Package Explanation

### Launch Files

#### navigate_to_goal.launch.py

This launch file sets up the parameters for the initial and goal poses and launches the `navigate_to_goal` node.

- **Arguments**:

  - `initial_x`: Initial X position (default: 0.0)
  - `initial_y`: Initial Y position (default: 0.0)
  - `initial_w`: Initial W orientation (default: 1.0)
  - `goal_x`: Goal X position (default: 1.0)
  - `goal_y`: Goal Y position (default: 1.0)
  - `goal_w`: Goal W orientation (default: 1.0)

- **Node**:
  - `navigate_to_goal`: Navigates the TurtleBot4 to the goal pose using the parameters provided.

### Source Files

#### navigate_to_goal.py

This node initializes the TurtleBot4's navigation, sets the initial pose, and navigates to the goal pose.

- **Parameters**:

  - `initial_x`: Initial X position
  - `initial_y`: Initial Y position
  - `initial_w`: Initial W orientation
  - `goal_x`: Goal X position
  - `goal_y`: Goal Y position
  - `goal_w`: Goal W orientation

- **Functionality**:
  - Sets the initial pose of the TurtleBot4.
  - Uses the TurtleBot4Navigator to navigate to the specified goal pose.
  - Logs a message when the goal is reached.

### setup.py

Defines the build and installation process for the package. Specifies entry points for the `navigate_to_goal` script.

### Directory Structure

The directory structure of the package is as follows:

```sh
saws_turtlebot4_nav2_move_to/
├── CMakeLists.txt
├── README.md
├── package.xml
├── resource/
│   └── saws_turtlebot4_nav2_move_to
├── launch/
│   └── navigate_to_goal.launch.py
├── saws_turtlebot4_nav2_move_to/
│   ├── __init__.py
│   └── navigate_to_goal.py
├── setup.cfg
└── setup.py
```

## References

- [TurtleBot4 Navigation](https://www.turtlebot.com/turtlebot4/)
- [ROS2 Documentation](https://docs.ros.org/en/foxy/index.html)
