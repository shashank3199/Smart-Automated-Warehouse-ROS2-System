FROM osrf/ros:humble-desktop-full

RUN apt update && apt install -y git wget curl python3-pip software-properties-common

RUN apt update
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN pip3 install setuptools==58.2.0

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install bootstrap tools
RUN apt update && apt install ros-humble-desktop -y \
    ros-humble-turtlebot4-desktop \
    ros-humble-control* \
    ros-humble-ros2-control* \
    ros-humble-moveit* \
    ros-humble-ros-ign* \
    ros-humble-joint-state-publisher-gui \
    ros-humble-kinematics-interface-kdl \
    ros-humble-rqt-joint-trajectory-controller \
    ros-humble-ur-description \
    ros-humble-aruco-ros \
    ros-humble-ur-moveit-config \
    ros-humble-ur-robot-driver \
    ros-humble-clearpath* \
    ros-humble-nav2-* \
    ros-humble-turtlesim \
    ros-humble-realsense2-* \
    ros-humble-librealsense2-* \
    ~nros-humble-rqt* \
    liborocos-bfl-dev \
    python3-colcon* \
    ignition-fortress \
    nano \
    less \
    xterm \
    ros-dev-tools

RUN apt update && apt install ros-humble-turtlebot4-description -y \
    ros-humble-turtlebot4-desktop \
    ros-humble-turtlebot4-bringup \
    ros-humble-turtlebot4-ignition-bringup \
    ros-humble-turtlebot4-ignition-gui-plugins \
    ros-humble-turtlebot4-ignition-toolbox \
    ros-humble-turtlebot4-msgs \
    ros-humble-turtlebot4-navigation \
    ros-humble-turtlebot4-node \
    ros-humble-turtlebot4-simulator \
    ros-humble-turtlebot4-viz

RUN rosdep update

RUN mkdir /etc/turtlebot4/
COPY ./turtlebot_setup.bash /etc/turtlebot4/setup.bash

# Entrypoint for image
COPY ./entrypoint.sh /entrypoint.sh
COPY ./.bashrc /root/.bashrc
RUN chmod 755 /entrypoint.sh
ENTRYPOINT [ "/entrypoint.sh" ]
