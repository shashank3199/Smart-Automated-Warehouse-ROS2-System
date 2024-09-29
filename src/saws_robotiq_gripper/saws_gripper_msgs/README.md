# SAWS Gripper Action Message Package

## Overview

`saws_gripper_msgs` is a ROS2 package that defines an action message for controlling a gripper. This package provides the message interface required for sending commands to and receiving feedback from a gripper control server.

## Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
  - [CMakeLists.txt](#cmakeliststxt)
  - [action/Gripper.action](#actiongripperaction)
  - [package.xml](#packagexml)
- [Usage](#usage)
  - [Clone the Repository](#clone-the-repository)
  - [Build the Package](#build-the-package)
  - [Source the Workspace](#source-the-workspace)
  - [Use the Action Message](#use-the-action-message)
- [Dependencies](#dependencies)

## Package Structure

The package consists of the following structure and key files:

```
saws_gripper_msgs/
├── CMakeLists.txt
├── action/
│   └── Gripper.action
└── package.xml
```

### CMakeLists.txt

**Key Configurations**:

- Finds necessary dependencies (`ament_cmake`, `rosidl_default_generators`).
- Generates ROS2 interfaces for the `Gripper` action.

### action/Gripper.action

**Purpose**: Defines the action message for controlling the gripper. The action message is split into three sections: Goal, Result, and Feedback.

**Sections**:

- **Goal**: Contains the command for the gripper.
- **Result**: Indicates the success of the command.
- **Feedback**: Provides status feedback of the gripper.

### package.xml

- Defines the package metadata, dependencies, and export information.
- Build dependencies (`ament_cmake`, `rosidl_default_generators`).
- Execution dependencies (`rosidl_default_runtime`).

## Usage

### Clone the Repository

Clone the repository containing `saws_gripper_msgs` into your ROS2 workspace.

### Build the Package

Navigate to the root of your ROS2 workspace and build the package using the following command:

```sh
colcon build --packages-select saws_gripper_msgs
```

### Source the Workspace

After building the package, source your ROS2 workspace to make the new message definitions available:

```sh
source install/setup.bash
```

### Use the Action Message

In your ROS2 nodes, import the `Gripper` action definition and use it to send commands to the gripper control server:

```python
from saws_gripper_msgs.action import Gripper
```

## Dependencies

The package depends on the following ROS2 packages:

- `ament_cmake`
- `rosidl_default_generators`
- `rosidl_default_runtime`
