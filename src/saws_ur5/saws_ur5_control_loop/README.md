# SAWS UR5 Control Loop

## Overview

`saws_ur5_control_loop` is a ROS2 package designed to manage the control loop for a UR5 robot. It includes functionalities for moving the UR5 robot's joints based on sensor inputs and other conditions.

## Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
  - [package.xml](#packagexml)
  - [resource/saws_ur5_control_loop](#resourcesaws_ur5_control_loop)
  - [saws_ur5_control_loop/\_\_init\_\_.py](#saws_ur5_control_loop___init__py)
  - [saws_ur5_control_loop/saws_ur5_control_loop.py](#saws_ur5_control_loopsaws_ur5_control_looppy)
  - [setup.cfg](#setupcfg)
  - [setup.py](#setuppy)
- [Class and Method Details](#class-and-method-details)
  - [Class: SawsControlLoop](#class-sawscontrolloop)
    - [Methods](#methods)
- [Usage](#usage)
  - [Clone the Repository](#clone-the-repository)
  - [Build the Package](#build-the-package)
  - [Source the Workspace](#source-the-workspace)
  - [Run the Control Loop Node](#run-the-control-loop-node)
- [Dependencies](#dependencies)

## Package Structure

The package consists of the following structure and key files:

```
saws_ur5_control_loop/
├── package.xml
├── resource/
│   └── saws_ur5_control_loop
├── saws_ur5_control_loop/
│   ├── __init__.py
│   └── saws_ur5_control_loop.py
├── setup.cfg
└── setup.py
```

### package.xml

- **Purpose**: Defines the package metadata, dependencies, and export information.
- **Key Elements**:
  - Package name, version, description, maintainer, and license.
  - Testing dependencies (`ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`).
  - Exported build type (`ament_python`).

### resource/saws_ur5_control_loop

- **Purpose**: A marker file used by the package management system to identify the package.

### saws_ur5_control_loop/**init**.py

- **Purpose**: Marks the directory as a Python package.

### saws_ur5_control_loop/saws_ur5_control_loop.py

- **Purpose**: Contains the implementation of the control loop for the UR5 robot.
- **Class Definition**:
  - **SawsControlLoop**:
    - **Methods**:
      - `__init__(self)`: Initializes the node, publishers, subscribers, and state variables.
      - `joint_state_callback(self, msg)`: Callback to update the current joint state.
      - `gripper_picked_status_callback(self, msg)`: Callback to handle the gripper picked status.
      - `turtlebot_arrived_callback(self, msg)`: Callback to handle the turtlebot arrival status.
      - `move_shoulder_pan_joint(self, angle)`: Moves the shoulder pan joint to a specified angle.
      - `note_transform(self)`: Notes the current transform from base_link to gripper_pick.
      - `create_transform_matrix(self, translation, rotation)`: Creates a transformation matrix from translation and rotation (quaternion).
      - `publish_pickup_block(self, flag)`: Publishes the pickup block flag.
      - `send_setpoint_to_noted_transform(self)`: Sends the setpoint to the noted transform.

### setup.cfg

- **Purpose**: Configures the installation paths for the package.
- **Key Elements**:
  - Specifies the script directory for development and installation.

### setup.py

- **Purpose**: Defines the setup configuration for the Python package.
- **Key Elements**:
  - Package name, version, and maintainer information.
  - Data files to be installed (`package.xml` and resource file).
  - Entry points for the console scripts.

## Class and Method Details

### Class: SawsControlLoop

The `SawsControlLoop` class manages the control loop for the UR5 robot, including the handling of various sensor inputs and publishing appropriate commands to the robot's joints.

#### Methods

- **`__init__(self)`**: Constructor to initialize the node, publishers, subscribers, and state variables.
- **`joint_state_callback(self, msg)`**: Callback to update the current joint state from `/joint_states` topic.
- **`gripper_picked_status_callback(self, msg)`**: Callback to handle the gripper picked status from `/gripper_picked_status` topic. If the gripper has picked an object and the pickup block flag is set, it publishes the setpoint to the noted transform.
- **`turtlebot_arrived_callback(self, msg)`**: Callback to handle the turtlebot arrival status from `/turtlebot_arrived` topic. If the turtlebot has arrived, it moves the shoulder pan joint and notes the current transform.
- **`move_shoulder_pan_joint(self, angle)`**: Moves the shoulder pan joint to a specified angle and publishes the joint state.
- **`note_transform(self)`**: Notes the current transform from `base_link` to `gripper_pick` using the TF2 listener and buffer.
- **`create_transform_matrix(self, translation, rotation)`**: Creates a transformation matrix from translation and rotation (quaternion).
- **`publish_pickup_block(self, flag)`**: Publishes the pickup block flag to the `/pickup_block` topic.
- **`send_setpoint_to_noted_transform(self)`**: Sends the setpoint to the noted transform if available.

## Usage

### Clone the Repository

Clone the repository containing `saws_ur5_control_loop` into your ROS2 workspace.

### Build the Package

Navigate to the root of your ROS2 workspace and build the package using the following command:

```sh
colcon build --packages-select saws_ur5_control_loop
```

### Source the Workspace

After building the package, source your ROS2 workspace to make the new control loop node available:

```sh
source install/setup.bash
```

### Run the Control Loop Node

Use the following command to run the control loop node:

```sh
ros2 run saws_ur5_control_loop saws_ur5_control_loop
```

## Dependencies

The package depends on the following ROS2 packages:

- `rclpy`
- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `tf2_ros`
- `scipy`
- `numpy`
