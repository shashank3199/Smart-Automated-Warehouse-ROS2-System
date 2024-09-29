# Smart Automated Warehouse System (SAWS) Docker Setup

This README provides instructions on how to set up and use the Docker environment for the Smart Automated Warehouse System (SAWS) project. This environment is tailored for working with the TurtleBot4 robot and includes tools for teleoperation, localization, and SLAM.

## Index

- [Introduction](#smart-automated-warehouse-system-saws-docker-setup)
- [Index](#index)
- [Setup Guide](#setup-guide)
  - [Building the Docker Image](#building-the-docker-image)
  - [Running the Docker Container](#running-the-docker-container)
  - [Connecting to the same instance of the Docker Container](#connecting-to-the-same-instance-of-the-docker-container)
- [Explanation of Files](#explanation-of-files)
  - [`.bashrc`](#bashrc)
  - [`Dockerfile`](#dockerfile)
  - [`docker_build.sh`](#docker_buildsh)
  - [`docker_run.sh`](#docker_runsh)
  - [`entrypoint.sh`](#entrypointsh)
  - [`turtlebot_setup.bash`](#turtlebot_setupbash)
- [References](#references)

## Setup Guide

### Building the Docker Image

To build the Docker image for SAWS, use the `docker_build.sh` script. This script will create a Docker image named `saws:latest`.

```bash
./docker_build.sh
```

### Running the Docker Container

To run the Docker container, use the `docker_run.sh` script. This script prepares the Xauthority data for GUI applications and starts the container with the necessary environment variables and device permissions.

```bash
./docker_run.sh
```

### Connecting to the same instance of the Docker Container

To connect to a running Docker Container from another instance of terminal, find out the name of the current container -

```bash
docker ps -a
```

This will give a list of running containers in the following format -

```
CONTAINER ID   IMAGE         COMMAND                 CREATED         STATUS         PORTS     NAMES
3fc684f7251d   saws:latest   "/entrypoint.sh bash"   5 seconds ago   Up 4 seconds             saws-1
```

From this, determine the name (in this case `saws-1`) of the container you wish to connect to and run -

```bash
docker exec -it saws-1 bash
```

To use ROS2 functionality in this new terminal of the docker container, run -

```bash
source ros_entrypoint.sh
```

## Explanation of Files

### `.bashrc`

This file sets up the environment for ROS and SAWS by sourcing various setup scripts.

### `Dockerfile`

The Dockerfile defines the Docker image for the SAWS environment. It installs ROS Humble, TurtleBot4 packages, and other dependencies required for the project.

### `docker_build.sh`

A script to build the Docker image. It uses the Dockerfile to create an image named `saws:latest`.

### `docker_run.sh`

A script to run the Docker container. It sets up Xauthority data for GUI applications and starts the container with appropriate environment variables and device permissions.

### `entrypoint.sh`

The entrypoint script for the Docker container. It sources the ROS installation and sets up the ROS environment before starting the specified shell.

### `turtlebot_setup.bash`

This setup script is sourced during the container startup to configure the TurtleBot4 environment variables and ROS domain ID.

## References

- [TurtleBot4 Navigation](https://www.turtlebot.com/turtlebot4/)
- [ROS2 Documentation](https://docs.ros.org/en/foxy/index.html)
- [Docker Documentation](https://docs.docker.com/)
