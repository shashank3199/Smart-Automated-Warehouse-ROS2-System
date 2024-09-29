# SAWS UR5 Resolve Rate Motion Control Package

The `saws_ur5_rrmc` package implements Resolve Rate Motion Control (RRMC) for the UR5 robot. This package provides the necessary nodes and functions to control the robot's motion using inverse kinematics, Jacobian calculations, and adjoint transformations.

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
colcon build --packages-select saws_ur5_rrmc
source install/setup.bash
```

### Running the Node

To run the node, use the following command:

```
ros2 run saws_ur5_rrmc saws_rrmc
```

## Directory Structure

The directory structure of the package is as follows:

```
saws_ur5_rrmc/
│
├── include/
│   └── saws_ur5_rrmc/
│       └── saws_rrmc.hpp
├── src/
│   └── saws_rrmc.cpp
├── CMakeLists.txt
├── package.xml
```

## File Roles

### `include/saws_ur5_rrmc/saws_rrmc.hpp`

This header file declares the `SawsResolveRateMotionControl` class and its member functions and variables. It includes necessary ROS 2 message types and Eigen for matrix operations.

### `src/saws_rrmc.cpp`

This source file implements the `SawsResolveRateMotionControl` class. It contains the definitions of the class's member functions, including callbacks for joint states and setpoints, as well as methods for performing forward kinematics, inverse kinematics, and Jacobian calculations.

### `CMakeLists.txt`

The CMake build script for the package. It defines the minimum CMake version, the project name, and its dependencies. It also specifies the executable target and its installation destination.

### `package.xml`

Defines the package and its dependencies, including `rclcpp`, `sensor_msgs`, `geometry_msgs`, `trajectory_msgs`, `tf2`, and `Eigen3`. It also specifies the build tool dependencies and any testing dependencies.

## Library Explanation

### Class Definitions

#### `SawsResolveRateMotionControl`

The main class implementing Resolve Rate Motion Control for the UR5 robot.

#### Private Members

##### Variables

- **`sub_setpoint`**: Subscriber for the setpoint topic.
- **`sub_jointstate`**: Subscriber for the joint states.
- **`pub_trajectory`**: Publisher for the joint trajectory.
- **`shoulder_pan_joint`**: Variable for the shoulder pan joint state.
- **`shoulder_lift_joint`**: Variable for the shoulder lift joint state.
- **`elbow_joint`**: Variable for the elbow joint state.
- **`wrist_1_joint`**: Variable for the wrist 1 joint state.
- **`wrist_2_joint`**: Variable for the wrist 2 joint state.
- **`wrist_3_joint`**: Variable for the wrist 3 joint state.

##### Member Functions

- **`JointState`**: Callback function for updating the current joint states.
- **`SetPoint`**: Callback function for handling new setpoints.
- **`Inverse`**: Computes the inverse of a 6x6 matrix.
- **`ForwardKinematics`**: Computes forward kinematics for the UR5 robot.
- **`ForwardKinematicsInverse`**: Computes inverse kinematics for the UR5 robot.
- **`Jacobian`**: Computes the Jacobian matrix for the UR5 robot.
- **`AdjointTransformationInverse`**: Computes the adjoint transformation inverse for a given transformation matrix.

#### Public Members

##### Constructors

- **`SawsResolveRateMotionControl`**: Initializes the node, sets up publishers and subscribers.

##### Member Functions

- **`main`**: Initializes and runs the node.

## References

- [ROS 2 Documentation](https://docs.ros.org/en/foxy/)
- [Eigen Library](https://eigen.tuxfamily.org/dox/)
- [UR5 Robot](https://www.universal-robots.com/products/ur5-robot/)
