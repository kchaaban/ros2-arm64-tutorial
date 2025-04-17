# 2. Setting up a ROS2 Workspace

After installing ROS2 Humble on your M1 Mac, the next step is to set up a ROS2 workspace. A workspace is a directory where you develop, build, and install ROS2 packages.

## Prerequisites

- Completed [Installation of ROS2 Humble](01_installation.md)
- Familiarity with basic terminal commands

## Understanding ROS2 Workspaces

A ROS2 workspace typically follows this structure:
- `src/`: Contains source code for packages
- `build/`: Contains intermediate build files
- `install/`: Contains installed targets
- `log/`: Contains build logs

## Creating a Workspace

1. Start your ROS2 Docker environment using the script created in the previous section:
   ```bash
   ~/start_ros2.sh
   