# Saws Gripper Actionlib Package

## Overview

`saws_gripper_actionlib` is a ROS2 package that provides an action server for controlling a gripper. The package includes the implementation of the action server, necessary message definitions, and launch files to start the server.

## Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
  - [CMakeLists.txt](#cmakeliststxt)
  - [include/saws_gripper_actionlib/saws_gripper_control.hpp](#includesaws_gripper_actionlibsaws_gripper_controlhpp)
  - [launch/saws_gripper_control.launch.py](#launchsaws_gripper_controllaunchpy)
  - [package.xml](#packagexml)
  - [src/saws_gripper_control.cpp](#srcsaws_gripper_controlcpp)
- [Class and Method Details](#class-and-method-details)
  - [Class: SawsGripperControl](#class-sawsgrippercontrol)
    - [Public Methods](#public-methods)
    - [Private Methods](#private-methods)
- [Usage](#usage)
  - [Clone the Repository](#clone-the-repository)
  - [Build the Package](#build-the-package)
  - [Source the Workspace](#source-the-workspace)
  - [Launch the Action Server](#launch-the-action-server)
- [Dependencies](#dependencies)

## Package Structure

The package consists of the following structure and key files:

```
saws_gripper_actionlib/
├── CMakeLists.txt
├── include/
│   └── saws_gripper_actionlib/
│       └── saws_gripper_control.hpp
├── launch/
│   └── saws_gripper_control.launch.py
├── package.xml
└── src/
    └── saws_gripper_control.cpp
```

### CMakeLists.txt

- Finds necessary dependencies (`ament_cmake`, `saws_gripper_msgs`, `rclcpp_action`, `control_msgs`, `rclcpp`, `sensor_msgs`).
- Specifies the executable for the action server.
- Installs the target executable and launch directory.

### include/saws_gripper_actionlib/saws_gripper_control.hpp

**Purpose**: Header file for the gripper control class, defining the interface and callbacks for the action server and client.

**Key Elements**:

- Includes necessary headers (`rclcpp`, `rclcpp_action`, `saws_gripper_msgs`, `control_msgs`, `sensor_msgs`).
- Defines the `SawsGripperControl` class with constants, action server/client, and subscription to joint state.
- Declares methods for handling goals, cancellations, acceptances, execution, and joint state callbacks.

### launch/saws_gripper_control.launch.py

**Purpose**: Launch file to start the gripper action server along with necessary transformations and driver launches.

**Key Elements**:

- Declares a launch argument for the serial port.
- Includes the gripper driver launch description.
- Defines a static transform publisher.
- Launches the gripper action server node.

### package.xml

**Purpose**: Defines the package metadata, dependencies, and export information.

**Key Elements**:

- Package name, version, description, maintainer, and license.
- Build dependencies (`ament_cmake`).
- Runtime dependencies (`saws_gripper_msgs`, `rclcpp_action`, `control_msgs`, `rclcpp`).
- Testing dependencies (`ament_lint_auto`, `ament_lint_common`).
- Exported build type (`ament_cmake`).

### src/saws_gripper_control.cpp

**Purpose**: Implementation file for the gripper control action server.

**Key Elements**:

- Includes the header file for the gripper control class.
- Implements the constructor, initializing action server, client, and joint state subscription.
- Defines callbacks for handling goals, cancellations, acceptances, and executions.
- Implements the joint state callback to update the current position of the gripper.
- Contains the main function to initialize and spin the ROS2 node.

## Class and Method Details

### Class: SawsGripperControl

The `SawsGripperControl` class manages the action server for controlling a gripper, handling goals, cancellations, and providing feedback.

#### Public Methods

- **`SawsGripperControl()`**: Constructor to initialize the node, action server, action client, and joint state subscription.
- **`handle_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<saws_gripper_msgs::action::Gripper>> goal_handle)`**: Callback to handle a new goal.
- **`handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<saws_gripper_msgs::action::Gripper>> goal_handle)`**: Callback to handle a goal cancellation.
- **`handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<saws_gripper_msgs::action::Gripper>> goal_handle)`**: Callback to handle an accepted goal.
- **`execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<saws_gripper_msgs::action::Gripper>> goal_handle)`**: Executes the accepted goal.
- **`joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)`**: Callback to update the current position of the gripper based on joint states.

#### Private Methods

- **`send_gripper_command(double position)`**: Sends a command to the gripper to move to the specified position.

## Usage

### Clone the Repository

Clone the repository containing `saws_gripper_actionlib` into your ROS2 workspace.

### Build the Package

Navigate to the root of your ROS2 workspace and build the package using the following command:

```sh
colcon build --packages-select saws_gripper_actionlib
```

### Source the Workspace

After building the package, source your ROS2 workspace to make the new action server available:

```sh
source install/setup.bash
```

### Launch the Action Server

Use the provided launch file to start the gripper action server:

```sh
ros2 launch saws_gripper_actionlib saws_gripper_control.launch.py
```

## Dependencies

The package depends on the following ROS2 packages:

- `ament_cmake`
- `saws_gripper_msgs`
- `rclcpp_action`
- `control_msgs`
- `rclcpp`
- `sensor_msgs`
