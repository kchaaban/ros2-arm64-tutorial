# Deploying to GitHub

This document provides step-by-step instructions for deploying this ROS2 ARM64 tutorial to your GitHub account.

## Prerequisites

- A GitHub account (in this example, we'll use "kchaaban")
- Git installed on your local machine
- Basic knowledge of Git commands

## Deployment Steps

### 1. Create a New Repository on GitHub

1. Go to [GitHub](https://github.com) and log in to your account
2. Click on the "+" icon in the top-right corner and select "New repository"
3. Name your repository (e.g., "ros2-arm64-tutorial")
4. Add an optional description
5. Choose whether to make it public or private
6. Click "Create repository"

### 2. Initialize Git in Your Local Project

Open a terminal in your project directory and run:

```bash
git init
```

### 3. Add and Commit Your Files

```bash
git add .
git commit -m "Initial commit"
```

### 4. Connect to Your GitHub Repository

Replace "kchaaban" with your GitHub username:

```bash
git remote add origin https://github.com/kchaaban/ros2-arm64-tutorial.git
git branch -M main
git push -u origin main
```

### 5. Verify Deployment

1. Go to your GitHub repository in a web browser
2. You should see all your project files uploaded
3. The README.md will be displayed on the repository's main page

## Viewing the Web Interface on GitHub Pages (Optional)

You can also deploy the web interface to GitHub Pages:

1. Go to your GitHub repository's "Settings" tab
2. Scroll down to "GitHub Pages" section
3. Under "Source", select "main" branch and "/docs" folder
4. Click "Save"

Note: You may need to move your web interface files to a `/docs` folder for this to work properly, or configure GitHub Pages to use the root directory.

## Keeping Your Repository Updated

After making changes to your project, update your GitHub repository with:

```bash
git add .
git commit -m "Description of changes"
git push
```

## Troubleshooting

If you encounter issues with the push operation, you may need to:

1. Ensure you have the correct repository URL
2. Check your GitHub credentials
3. Try using a personal access token if password authentication fails

For more help, refer to GitHub's documentation on [getting started with Git](https://docs.github.com/en/get-started/getting-started-with-git).