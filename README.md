# ROS2 Humble Simulation Tutorial for ARM64 Architecture

This repository contains a step-by-step tutorial for setting up and building a ROS2 Humble simulation project on ARM64 architecture (including Apple Silicon).

## Overview

ROS2 (Robot Operating System 2) is a set of software libraries and tools for building robot applications. Running ROS2 on ARM64 architecture requires specific considerations compared to traditional x86 platforms.

This tutorial guides you through:

1. [Installation of ROS2 Humble on ARM64](tutorial/01_installation.md)
2. [Setting up a ROS2 Workspace](tutorial/02_workspace_setup.md)
3. [Creating a ROS2 Package](tutorial/03_package_creation.md)
4. [Implementing Simulation Nodes](tutorial/04_simulation_nodes.md)
5. [Setting up Visualization](tutorial/05_visualization.md)
6. [Creating Launch Files](tutorial/06_launch_files.md)
7. [Building and Running the Simulation](tutorial/07_building_running.md)
8. [Troubleshooting ARM64-specific Issues](tutorial/08_troubleshooting.md)

## Project Structure

- `/tutorial/` - Step-by-step guide documents
- `/ros2_m1_sim/` - Example ROS2 package with simulation code
- `/templates/` - Web interface templates for browsing tutorials
- `main.py` - Flask web application for tutorial navigation

## Requirements

For using the tutorials and code:
- ARM64 device (Apple Silicon Mac, Raspberry Pi 4, etc.)
- Operating system compatible with ROS2 Humble
- At least 16GB of free disk space
- Basic knowledge of terminal commands

For running the web interface:
- Python 3.8+
- Flask
- Markdown2

## Getting Started

### Viewing Tutorials Locally

1. Clone this repository
2. Install the requirements: `pip install flask markdown2`
3. Run the web application: `python main.py`
4. Open your browser to http://localhost:5000

### Following Tutorials

Start with the first tutorial section: [Installation of ROS2 Humble on ARM64](tutorial/01_installation.md)

## Deploying to GitHub

To deploy this project to your GitHub account:

1. Create a new repository on GitHub (e.g., https://github.com/kchaaban/ros2-arm64-tutorial)
2. Initialize git in this project folder:
   ```bash
   git init
   git add .
   git commit -m "Initial commit"
   ```
3. Connect to your GitHub repository:
   ```bash
   git remote add origin https://github.com/kchaaban/ros2-arm64-tutorial.git
   git branch -M main
   git push -u origin main
   ```

## Web Interface

The included web interface allows you to:
- Browse all tutorial sections with navigation
- View source code with syntax highlighting
- Navigate the ROS2 package structure

## License

This project is licensed under the MIT License - see the LICENSE file for details.
