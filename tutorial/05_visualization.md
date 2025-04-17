# 5. Setting up Visualization

A key aspect of robotics simulation is visualization. This section will guide you through setting up visualization tools to view your ROS2 simulation on the M1 Mac.

## Prerequisites

- Completed [Implementing Simulation Nodes](04_simulation_nodes.md)
- ROS2 Humble Docker environment running

## ROS2 Visualization Tools

ROS2 offers several visualization tools:

1. **RViz2**: The primary 3D visualization tool for ROS2
2. **rqt**: A Qt-based framework for ROS GUIs
3. **rqt_graph**: Tool to visualize the ROS2 computation graph
4. **turtlesim**: Simple 2D simulator useful for basic demonstrations

For our simulation, we'll use rqt for debugging and system analysis, and we'll add a simple visualization node to demonstrate how the robot moves.

## Setting up RQT

RQT is a powerful tool for debugging and analyzing ROS2 systems. Let's use it to monitor our simulation:

1. Make sure you have sourced your workspace:
   ```bash
   source /root/ros2_humble_ws/install/setup.bash
   