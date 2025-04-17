# 1. Installing ROS2 Humble on M1 Apple Hardware

This guide covers the installation of ROS2 Humble on Apple M1 (ARM architecture) hardware. The installation process differs from traditional x86 installations due to the ARM architecture.

## Prerequisites

- Apple M1, M1 Pro, M1 Max, M1 Ultra, M2, or newer Apple Silicon Mac
- macOS Monterey (12) or newer
- At least 16GB of free disk space
- Administrator access to your system

## Installation Options

There are three primary methods for installing ROS2 Humble on M1 Macs:

1. Using a Docker container (recommended)
2. Using Rosetta 2 with binary installation
3. Building from source (most complex but best native performance)

In this tutorial, we'll focus on the Docker approach as it provides the best balance of ease and functionality.

## Setting Up Docker on M1 Mac

1. Download and install Docker Desktop for Apple Silicon from [Docker's website](https://www.docker.com/products/docker-desktop/)

2. Launch Docker Desktop and ensure it's running properly

3. Open Terminal and verify Docker installation:
   ```bash
   docker --version
   ```

## Creating a ROS2 Humble Docker Container

1. Pull the ROS2 Humble image for ARM64:
   ```bash
   docker pull osrf/ros:humble-desktop
   ```

2. Create a directory for ROS2 development on your host machine:
   ```bash
   mkdir -p ~/ros2_humble_ws
   ```

3. Run the container with GUI support and mount your workspace:
   ```bash
   xhost +local:docker
   
   docker run -it --name ros2_humble \
     --network=host \
     -e DISPLAY=$DISPLAY \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     -v ~/ros2_humble_ws:/root/ros2_humble_ws \
     osrf/ros:humble-desktop
   