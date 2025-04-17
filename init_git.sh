#!/bin/bash

# Simple script to initialize a git repository and prepare for GitHub deployment
# to the kchaaban account

echo "Initializing git repository for ROS2 ARM64 Tutorial..."

# Initialize git repository
git init

# Add all files
git add .

# Commit changes
git commit -m "Initial commit of ROS2 ARM64 Tutorial"

# Set up the remote repository (replace with your actual repository URL)
echo "Setting up remote repository..."
git remote add origin https://github.com/kchaaban/ros2-arm64-tutorial.git

# Set main as the default branch
git branch -M main

echo "Repository initialized and configured. Run the following command to push to GitHub:"
echo "git push -u origin main"
echo ""
echo "Note: You may need to authenticate with GitHub when pushing."