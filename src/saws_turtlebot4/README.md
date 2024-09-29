# SAWS TurtleBot4

The packages in this directory are to be placed in a ROS workspace on the TurtleBot4. These packages are essential for setting up and operating the Smart Automated Warehouse System (SAWS) with TurtleBot4, enabling functionalities such as teleoperation, localization, and SLAM.

## Setup Instructions

1. **Create a ROS Workspace**

   Create a ROS workspace on your TurtleBot4 if you don't already have one. A typical workspace structure is as follows:

   ```bash
   mkdir -p ~/saws_workspace/src
   cd ~/saws_workspace/src
   ```

2. **Clone the Packages**

   Clone the packages from this directory into the `src` folder of your ROS workspace:

   ```bash
   cd ~/saws_workspace/src
   git clone https://github.com/jhu-rsp/Smart-Automated-Warehouse-System.git
   ```

3. **Build the Workspace**

   After cloning the packages, build your workspace:

   ```bash
   cd ~/saws_workspace
   colcon build
   ```

4. **Source the Workspace**

   Source the setup script to overlay the workspace on your ROS environment:

   ```bash
   source ~/saws_workspace/install/setup.bash
   ```

## References

For more information on setting up and using ROS workspaces, refer to the following resources:

- [ROS2 Documentation](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)
- [TurtleBot4 Documentation](https://www.turtlebot.com/turtlebot4/)
