# ROS2 ARM64 Tutorial - Export Guide

This document provides instructions for using the exported ZIP file of the ROS2 ARM64 Tutorial project.

## Contents of Export

The exported ZIP file contains:

- Complete Flask web application for browsing the tutorials
- All tutorial markdown documentation files
- Full ROS2 simulation package source code
- GitHub deployment instructions and scripts
- Project configuration files

## Setting Up the Project Locally

### Option 1: Running the Web Interface

1. Extract the ZIP file to a directory of your choice:
   ```bash
   unzip ros2-arm64-tutorial.zip -d ros2-arm64-tutorial
   cd ros2-arm64-tutorial
   ```

2. Install the required Python packages:
   ```bash
   pip install flask markdown2 gunicorn
   ```

3. Run the Flask application:
   ```bash
   python main.py
   ```

4. Open your browser to http://localhost:5000 to view the tutorials

### Option 2: Deploying to GitHub

1. Extract the ZIP file to a directory of your choice
2. Create a new GitHub repository (e.g., at github.com/kchaaban/ros2-arm64-tutorial)
3. Run the included init_git.sh script:
   ```bash
   ./init_git.sh
   ```
4. Push the repository to GitHub:
   ```bash
   git push -u origin main
   ```

## Project Structure

- `/tutorial/` - All tutorial markdown files
- `/templates/` - Web interface HTML templates
- `/ros2_m1_sim/` - Complete ROS2 package with simulation code
- `main.py` - Flask web application
- `README.md` - Project overview
- `GITHUB_DEPLOYMENT.md` - Detailed GitHub deployment instructions

## Additional Information

For a complete overview of the project and its deployment status, refer to the DEPLOYMENT_SUMMARY.md file included in the export.

## License

This project is licensed under the MIT License - see the LICENSE file for details.