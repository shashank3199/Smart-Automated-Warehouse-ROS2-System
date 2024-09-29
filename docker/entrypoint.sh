#!/bin/bash

# Entrypoint for the ROS SAWS Docker container.
# echo 'source "/opt/ros/humble/setup.bash"' >> ~/.bashrc

# ==== Get the shell environment ====
shell_path=`which $1`
if [ -z $shell_path ]; then
    echo "Shell $1 not found in the container"
    exit 1
else
    echo "Shell path: ${shell_path}"
fi

# Source ROS installation
source_ros_installation() {
    echo "##### Sourcing ROS Installation #####"
    source /opt/ros/humble/setup.bash
    cd /home/shared
}

# Show the package path using ROS_PACKAGE_PATH
show_package_path() {
    echo "##### ROS Package Path #####"
    echo "  " $ROS_PACKAGE_PATH | sed 's/\:/\n   /g'
    echo ""
}

# ==== Main calls to the functions ====
# ROS installation
source_ros_installation
# Package path
show_package_path

# ==== Execute =====
echo -e "Starting ${shell_path}...\n\n"
exec "${shell_path}"
