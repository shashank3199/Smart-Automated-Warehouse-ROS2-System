# SAWS UR5 Joint Interpolator

This package, `saws_ur5_joint_interpolator`, provides functionality for interpolating between two joint states of a UR5 robot. It is implemented using ROS 2 and ensures smooth transitions between different joint configurations.

## Index

- [Index](#index)
- [User Guide](#user-guide)
  - [Downloading the Library](#downloading-the-library)
  - [Building the Package](#building-the-package)
  - [Running the Node](#running-the-node)
- [Directory Structure](#directory-structure)
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

To download the library, you can clone the repository using the following command:

```
git clone https://github.com/jhu-rsp/Smart-Automated-Warehouse-System.git
```

### Building the Package

To build the package, navigate to the root directory of the package and run the following commands:

```
colcon build --packages-select saws_ur5_joint_interpolator
source install/setup.bash
```

### Running the Node

To run the node, use the following command:

```
ros2 run saws_ur5_joint_interpolator saws_ur5_joint_interpolator_node
```

## Directory Structure

The directory structure of the package is as follows:

```
saws_ur5_joint_interpolator/
│
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── saws_ur5_joint_interpolator/
│       └── saws_ur5_joint_interpolator.hpp
├── src/
│   └── saws_ur5_joint_interpolator.cpp
└── launch/
    └── saws_ur5_joint_interpolator.launch.py
```

### File Roles

- **CMakeLists.txt**: Contains the build configuration for the package.
- **package.xml**: Provides meta-information about the package, such as its name, version, dependencies, etc.
- **README.md**: The file you are reading now, which provides an overview and usage instructions for the package.
- **include/saws_ur5_joint_interpolator/saws_ur5_joint_interpolator.hpp**: The header file containing the class definition for `SawsUR5JointInterpolator`.
- **src/saws_ur5_joint_interpolator.cpp**: The source file containing the implementation of the `SawsUR5JointInterpolator` class.
- **launch/saws_ur5_joint_interpolator.launch.py**: A launch file to start the node using ROS 2 launch system.

## Library Explanation

### Class Definitions

The primary class in this package is `SawsUR5JointInterpolator`, which handles the interpolation between two joint states for the UR5 robot.

#### Private Members

##### Variables

- **target*joint_subscription***: A ROS subscription for receiving the target joint states.
- **current*joint_subscription***: A ROS subscription for receiving the current joint states.
- **trajectory*publisher***: A ROS publisher for sending the interpolated joint trajectories.
- **timer\_**: A ROS timer for periodically publishing the trajectory.
- **target*joint_state***: Stores the target joint state.
- **current*joint_state***: Stores the current joint state.
- **target*received***: A flag indicating if the target joint state has been received.
- **current*received***: A flag indicating if the current joint state has been received.

##### Member Functions

- **target_joint_callback**: Callback function to handle incoming target joint state messages.
- **current_joint_callback**: Callback function to handle incoming current joint state messages.
- **publish_trajectory**: Computes and publishes the interpolated joint trajectory based on the target and current joint states.

#### Public Members

##### Constructors

- **SawsUR5JointInterpolator()**: Initializes the node, subscriptions, publisher, and timer.

##### Member Functions

- **target_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)**: Handles the target joint state messages by storing the received data and setting the target*received* flag to true.
- **current_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)**: Handles the current joint state messages by storing the received data and setting the current*received* flag to true.
- **publish_trajectory()**: Publishes the interpolated joint trajectory based on the received target and current joint states. It calculates the trajectory points and sends them to the trajectory publisher.

## References

- [ROS 2 Documentation](https://docs.ros.org/en/foxy/index.html)
- [UR5 Robot User Manual](https://www.universal-robots.com/download/)

This README provides a comprehensive overview of the `saws_ur5_joint_interpolator` package, including its directory structure, file roles, and detailed explanations of its classes and members.
