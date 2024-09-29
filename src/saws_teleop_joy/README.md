# SAWS Teleop Joy Package

## Overview

`saws_teleop_joy` is a ROS2 package designed to enable teleoperation of a UR5 robot and Robotiq gripper using a joystick. The package includes implementations for Cartesian and joint control of the UR5 robot, as well as control of the Robotiq gripper.

## Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
  - [CMakeLists.txt](#cmakeliststxt)
  - [include/saws_teleop_joy/ps4_macros.hpp](#includesaws_teleop_joyps4_macroshpp)
  - [include/saws_teleop_joy/saws_gripper_control.hpp](#includesaws_teleop_joysaws_gripper_controlhpp)
  - [include/saws_teleop_joy/saws_ur5_cartesian_control.hpp](#includesaws_teleop_joysaws_ur5_cartesian_controlhpp)
  - [include/saws_teleop_joy/saws_ur5_joint_control.hpp](#includesaws_teleop_joysaws_ur5_joint_controlhpp)
  - [launch/saws_teleop_gripper.launch.py](#launchsaws_teleop_gripperlaunchpy)
  - [launch/saws_teleop_ur5.launch.py](#launchsaws_teleop_ur5launchpy)
  - [package.xml](#packagexml)
  - [src/saws_gripper_control.cpp](#srcsaws_gripper_controlcpp)
  - [src/saws_ur5_cartesian_control.cpp](#srcsaws_ur5_cartesian_controlcpp)
  - [src/saws_ur5_joint_control.cpp](#srcsaws_ur5_joint_controlcpp)
- [Usage](#usage)
  - [Clone the Repository](#clone-the-repository)
  - [Build the Package](#build-the-package)
  - [Source the Workspace](#source-the-workspace)
  - [Launch the Teleoperation Nodes](#launch-the-teleoperation-nodes)
- [Dependencies](#dependencies)
- [Control Mapping](#control-mapping)
  - [Gripper Control](#gripper-control)
  - [Joint Control](#joint-control)
  - [Cartesian Control](#cartesian-control)

## Package Structure

The package consists of the following structure and key files:

```
saws_teleop_joy/
├── CMakeLists.txt
├── include/
│   └── saws_teleop_joy/
│       ├── ps4_macros.hpp
│       ├── saws_gripper_control.hpp
│       ├── saws_ur5_cartesian_control.hpp
│       └── saws_ur5_joint_control.hpp
├── launch/
│   ├── saws_teleop_gripper.launch.py
│   └── saws_teleop_ur5.launch.py
├── package.xml
└── src/
    ├── saws_gripper_control.cpp
    ├── saws_ur5_cartesian_control.cpp
    └── saws_ur5_joint_control.cpp
```

### CMakeLists.txt

- Sets project details and compilation options.
- Finds necessary dependencies (`ament_cmake`, `rclcpp`, `joy`, `sensor_msgs`, `geometry_msgs`, `tf2_ros`, `control_msgs`, `rclcpp_action`).
- Adds executables for Cartesian control, joint control, and gripper control.
- Installs the target executables and launch directory.

### include/saws_teleop_joy/ps4_macros.hpp

**Purpose**: Defines macros for PS4 joystick button and axis mappings.

**Macros**:

- Button mappings (e.g., `PS4_BUTTON_X`, `PS4_BUTTON_O`, etc.).
- Axis mappings (e.g., `PS4_AXIS_LEFT_X`, `PS4_AXIS_LEFT_Y`, etc.).

### include/saws_teleop_joy/saws_gripper_control.hpp

**Purpose**: Header file for the gripper control class, defining the interface and callbacks for controlling the Robotiq gripper using a joystick.

**Class Definition**:

- **JoyRobotiqGripperControl**:
  - **Public Methods**:
    - `JoyRobotiqGripperControl()`: Constructor to initialize the node, subscriptions, and action client.
  - **Private Methods**:
    - `void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)`: Callback for joystick inputs.
    - `void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)`: Callback for joint state updates.
    - `void send_gripper_command(double position)`: Function to send commands to the gripper.

### include/saws_teleop_joy/saws_ur5_cartesian_control.hpp

**Purpose**: Header file for the Cartesian control class, defining the interface and callbacks for controlling the UR5 end-effector position using a joystick.

**Class Definition**:

- **JoyUR5CartesianControl**:
  - **Public Methods**:
    - `JoyUR5CartesianControl()`: Constructor to initialize the node, subscriptions, publisher, and TF2 listener.
  - **Private Methods**:
    - `void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)`: Callback for joystick inputs.
    - `void update_transform()`: Function to update the transform of the end-effector.

### include/saws_teleop_joy/saws_ur5_joint_control.hpp

**Purpose**: Header file for the joint control class, defining the interface and callbacks for controlling the UR5 joints using a joystick.

**Class Definition**:

- **JoyUR5JointControl**:
  - **Public Methods**:
    - `JoyUR5JointControl()`: Constructor to initialize the node, subscriptions, and publisher.
  - **Private Methods**:
    - `void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)`: Callback for joint state updates.
    - `void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)`: Callback for joystick inputs.

### launch/saws_teleop_gripper.launch.py

**Purpose**: Launch file to start the gripper teleoperation node along with the necessary transformations and driver launches.

**Key Elements**:

- Declares a launch argument for the joystick device.
- Launches the `joy` node for joystick inputs.
- Includes the gripper control launch description.
- Launches the gripper teleoperation node.

### launch/saws_teleop_ur5.launch.py

**Purpose**: Launch file to start the UR5 teleoperation node with options for Cartesian or joint control.

**Key Elements**:

- Declares launch arguments for joystick device and control type.
- Launches the `joy` node for joystick inputs.
- Includes the UR5 bringup launch description.
- Launches the Cartesian or joint control node based on the control type argument.

### package.xml

- Build dependencies (`ament_cmake`, `rclcpp`, `joy`, `geometry_msgs`, `sensor_msgs`, `tf2_ros`, `control_msgs`, `rclcpp_action`).
- Runtime dependencies (`rclcpp`, `joy`, `geometry_msgs`, `sensor_msgs`, `tf2_ros`, `control_msgs`, `rclcpp_action`).

### src/saws_gripper_control.cpp

**Purpose**: Implementation file for the gripper control node.

**Class Definition**:

- **JoyRobotiqGripperControl**:
  - Implements the constructor, initializing subscriptions and action client.
  - Defines the joystick callback to open/close the gripper based on button inputs.
  - Defines the joint state callback to update the current position of the gripper.
  - Implements the function to send gripper commands via the action client.

### src/saws_ur5_cartesian_control.cpp

**Purpose**: Implementation file for the Cartesian control node.

**Class Definition**:

- **JoyUR5CartesianControl**:
  - Implements the constructor, initializing subscriptions, publisher, and TF2 listener.
  - Defines the joystick callback to update the end-effector position based on joystick inputs.
  - Implements the function to update the transform of the end-effector based on the current position.

### src/saws_ur5_joint_control.cpp

**Purpose**: Implementation file for the joint control node.

**Class Definition**:

- **JoyUR5JointControl**:
  - Implements the constructor, initializing subscriptions and publisher.
  - Defines the joint state callback to update the current joint positions.
  - Defines the joystick callback to update the joint positions based on joystick inputs.

## Usage

To use the `saws_teleop_joy` package in your ROS2 workspace, follow these steps:

### Clone the Repository

Clone the repository containing `saws_teleop_joy` into your ROS2 workspace.

### Build the Package

Navigate to the root of your ROS2 workspace and build the package using the following command:

```sh
colcon build --packages-select saws_teleop_joy
```

### Source the Workspace

After building the package, source your ROS2 workspace to make the new teleoperation nodes available:

```sh
source install/setup.bash
```

### Launch the Teleoperation Nodes

Use the provided launch files

to start the teleoperation nodes:

For gripper teleoperation:

```sh
ros2 launch saws_teleop_joy saws_teleop_gripper.launch.py
```

For UR5 teleoperation (Cartesian or joint control):

```sh
ros2 launch saws_teleop_joy saws_teleop_ur5.launch.py control_type:=cartesian
```

or

```sh
ros2 launch saws_teleop_joy saws_teleop_ur5.launch.py control_type:=joint
```

## Dependencies

The package depends on the following ROS2 packages:

- `ament_cmake`
- `rclcpp`
- `joy`
- `geometry_msgs`
- `sensor_msgs`
- `tf2_ros`
- `control_msgs`
- `rclcpp_action`

## Control Mapping

The following tables provide an overview of the joystick controls for the UR5 robot and Robotiq gripper:

### Gripper Control

| Control Action | Joystick Button |
| -------------- | --------------- |
| Open Gripper   | L1 Button       |
| Close Gripper  | R1 Button       |

### Joint Control

| Control Action      | Joystick Button/Axis |
| ------------------- | -------------------- |
| Rotate Wrist 3      | L1 / R1 Buttons      |
| Rotate Wrist 2      | L2 / R2 Axes         |
| Rotate Wrist 1      | Left Stick Y-Axis    |
| Move Elbow          | Right Stick Y-Axis   |
| Move Shoulder Lift  | Left Stick X-Axis    |
| Rotate Shoulder Pan | Right Stick X-Axis   |

### Cartesian Control

| Control Action        | Joystick Button/Axis |
| --------------------- | -------------------- |
| Move End-Effector (X) | Left Stick X-Axis    |
| Move End-Effector (Y) | Left Stick Y-Axis    |
| Move End-Effector (Z) | Up/Down Buttons      |
