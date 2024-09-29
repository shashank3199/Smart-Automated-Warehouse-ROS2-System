# SAWS TurtleBot4 Navigation Bringup

This package is part of the Smart Automated Warehouse System (SAWS) and provides the necessary launch files to bring up the TurtleBot4 with navigation capabilities. It localizes the robot using a saved map and navigates through predefined positions sequentially.

## Index

- [Introduction](#introduction)
- [Index](#index)
- [User Guide](#user-guide)
  - [Usage](#usage)
- [Package Explanation](#package-explanation)
  - [Launch Files](#launch-files)
- [References](#references)

## User Guide

### Usage

To bring up the TurtleBot4 with navigation capabilities, use the following command:

```sh
ros2 launch saws_turtlebot4_nav2_bringup saws_turtlebot4_nav2_bringup.launch.py
```

This command localizes the TurtleBot4 using a saved map and navigates through the positions saved earlier using the `saws_turtlebot4_teleop_setup` package.

## Package Explanation

### Launch Files

#### saws_turtlebot4_nav2_bringup.launch.py

This launch file sets up the necessary nodes and parameters to bring up the TurtleBot4 with navigation capabilities. It performs the following actions:

- **Includes**:

  - `nav2.launch.py` from the `turtlebot4_navigation` package to bring up the navigation stack.
  - `saws_turtlebot4_teleop_setup.launch.py` with `localization:=true` and the `map` argument to set up localization.

- **Reads**:

  - `positions.yaml` from the `saws_turtlebot4_teleop_setup` package to get the predefined positions for navigation.
  - `wyman160.yaml` from the `saws_turtlebot4_teleop_setup` package as the map file for localization.

- **Sequences**:
  - Navigates through each position sequentially by calling the `navigate_to_goal.launch.py` file from the `saws_turtlebot4_nav2_move_to` package.
  - Logs navigation steps and adds delays between goals to allow the TurtleBot4 to reach each position.

## References

- [TurtleBot4 Navigation](https://www.turtlebot.com/turtlebot4/)
- [ROS2 Documentation](https://docs.ros.org/en/foxy/index.html)

## Directory Structure

The directory structure of the package is as follows:

```sh
saws_turtlebot4_nav2_bringup/
├── CMakeLists.txt
├── README.md
├── package.xml
└── launch/
    └── saws_turtlebot4_nav2_bringup.launch.py
```

### File Roles

- **CMakeLists.txt**: Contains the build configuration for the package.
- **README.md**: The file you are reading now, which provides an overview and usage instructions for the package.
- **package.xml**: Provides meta-information about the package, such as its name, version, dependencies, etc.
- **launch/saws_turtlebot4_nav2_bringup.launch.py**: A launch file to start the node using the ROS 2 launch system.
