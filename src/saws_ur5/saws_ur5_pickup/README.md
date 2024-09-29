# SAWS UR5 Pickup Package

The `saws_ur5_pickup` package enables the UR5 robot to pick up objects using a combination of camera inputs and predefined transformation matrices. It integrates with the RealSense camera, the `aruco_ros` package for marker detection, and the `saws_ur5_rrmc` package for robot control.

## Index

- [Index](#index)
- [User Guide](#user-guide)
  - [Downloading the Library](#downloading-the-library)
  - [Building the Package](#building-the-package)
  - [Running the Node](#running-the-node)
- [Directory Structure](#directory-structure)
- [File Roles](#file-roles)
- [Library Explanation](#library-explanation)
  - [Class Definitions](#class-definitions)
    - [Private Members](#private-members)
      - [Variables](#variables)
      - [Member Functions](#member-functions)
    - [Public Members](#public-members)
      - [Constructors](#constructors)
      - [Member Functions](#member-functions)
- [References](#references)

## User Guide

### Downloading the Library

To download the library, clone the repository using the following command:

```
git clone https://github.com/jhu-rsp/Smart-Automated-Warehouse-System.git
```

### Building the Package

Navigate to the root directory of the package and run the following commands:

```
colcon build --packages-select saws_ur5_pickup
source install/setup.bash
```

### Running the Node

To run the node, use the following command:

```
ros2 launch saws_ur5_pickup saws_pickup.launch.py
```

## Directory Structure

The directory structure of the package is as follows:

```
saws_ur5_pickup/
│
├── config/
│   └── handeye.yaml
├── launch/
│   └── saws_pickup.launch.py
├── resource/
│   └── saws_ur5_pickup
├── saws_ur5_pickup/
│   ├── __init__.py
│   └── saws_pickup.py
├── setup.cfg
├── setup.py
└── package.xml
```

## File Roles

### `config/handeye.yaml`

This file contains the configuration for the hand-eye calibration, specifying the translation and rotation (in quaternion format) required to transform the coordinates between the camera and the robot's end effector.

### `launch/saws_pickup.launch.py`

The launch file sets up the nodes required for the UR5 pickup operation. It includes the UR5 robot bring-up, RealSense camera node, `aruco_ros` node for marker detection, and the `saws_ur5_pickup` node for executing the pickup logic.

### `resource/saws_ur5_pickup`

A resource file indicating the presence of the package in the ROS 2 environment.

### `saws_ur5_pickup/__init__.py`

This is an empty file that marks the directory as a Python package.

### `saws_ur5_pickup/saws_pickup.py`

This Python script defines the `SawsPickup` class, which implements the logic for picking up objects using the UR5 robot. It handles subscribers, publishers, and transforms necessary for the operation.

### `setup.cfg`

Configuration file for setting up the package.

### `setup.py`

Setup script for installing the package. It specifies the package details and entry points.

### `package.xml`

Defines the package and its dependencies, including `ur_robot_driver`, `realsense2_camera`, `saws_ur5_rrmc`, and `aruco_ros`.

## Library Explanation

### Class Definitions

#### `SawsPickup`

The main class implementing the pickup logic for the UR5 robot.

#### Private Members

##### Variables

- **`self.publisher_`**: Publisher for the setpoint topic.
- **`self.joint_publisher_`**: Publisher for the joint state topic.
- **`self.gripper_status_publisher_`**: Publisher for the gripper status topic.
- **`self.joint_state_subscriber`**: Subscriber for the joint states.
- **`self.pickup_block_subscriber`**: Subscriber for the pickup block status.
- **`self.tf_buffer`**: Buffer for storing TF transforms.
- **`self.tf_listener`**: Listener for TF transforms.
- **`self.current_joint_state`**: Stores the current joint state.
- **`self.pickup_block_flag`**: Flag indicating whether a pickup operation should be performed.
- **`self.config_transform`**: Transformation matrix from the hand-eye calibration configuration.
- **`self.gripper_client_`**: Action client for controlling the gripper.
- **`self.yaw_alignment_enabled`**: Flag to enable yaw alignment during pickup.

##### Member Functions

- **`create_transform_matrix`**: Creates a transformation matrix from translation and rotation values.
- **`joint_state_callback`**: Callback function for updating the current joint state.
- **`pickup_block_callback`**: Callback function for handling the pickup block status.
- **`perform_pickup_operations`**: Performs the pickup operations using TF transforms and publishes commands.
- **`send_gripper_command`**: Sends a command to close the gripper using an action client.

#### Public Members

##### Constructors

- **`__init__`**: Initializes the node, sets up publishers, subscribers, TF buffer and listener, loads the configuration, and initializes the action client.

##### Member Functions

- **`main`**: Initializes and runs the node.

## References

- [UR Robot Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [RealSense Camera](https://github.com/IntelRealSense/realsense-ros)
- [aruco_ros](https://github.com/pal-robotics/aruco_ros)
- [saws_ur5_rrmc](https://github.com/your-repository/saws_ur5_rrmc)
