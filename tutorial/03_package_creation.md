# 3. Creating a ROS2 Package

Now that you have a ROS2 workspace set up, it's time to create a package for our simulation. In ROS2, packages are the organizational unit for your code, which can contain nodes, libraries, datasets, or other ROS2-related files.

## Prerequisites

- Completed [Setting up a ROS2 Workspace](02_workspace_setup.md)
- Basic understanding of Python (as we'll be creating Python-based nodes)

## Package Types in ROS2

ROS2 supports two primary types of packages:

1. **CMake packages**: Used for C++ code or mixed C++/Python code
2. **Python packages**: Easier to set up for Python-only code

For our simulation, we'll create a Python package since we'll be writing our nodes in Python.

## Creating a New Package

1. Start your ROS2 Docker environment and navigate to your workspace:
   ```bash
   cd /root/ros2_humble_ws
   ```

2. Create a new Python package:
   ```bash
   ros2 pkg create --build-type ament_python --node-name robot_publisher ros2_m1_sim
   ```

   This command creates a new package named `ros2_m1_sim` with an initial node called `robot_publisher`.

3. Examine the package structure:
   ```bash
   ls -la src/ros2_m1_sim/
   ```

   You should see the following structure:
   ```
   ros2_m1_sim/
   ├── package.xml
   ├── resource/
   ├── ros2_m1_sim/
   │   ├── __init__.py
   │   └── robot_publisher.py
   ├── setup.cfg
   ├── setup.py
   └── test/
   ```

## Understanding Package Files

### package.xml

This file contains metadata about your package, such as dependencies, version, and maintainer information.

Open the file to see its contents:
```bash
cat src/ros2_m1_sim/package.xml
