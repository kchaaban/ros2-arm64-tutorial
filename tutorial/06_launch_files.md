# 6. Creating Launch Files

ROS2 launch files allow you to start multiple nodes simultaneously with configured parameters. In this section, we'll create launch files to simplify the process of starting our simulation.

## Prerequisites

- Completed [Setting up Visualization](05_visualization.md)
- Understanding of ROS2 launch files

## Understanding ROS2 Launch Files

ROS2 uses Python-based launch files (instead of XML files used in ROS1). Launch files:

- Start multiple nodes at once
- Configure node parameters
- Set up remappings for topics, services, etc.
- Execute commands as part of the launch process

## Creating a Basic Launch File

Let's create a launch file for our simulation:

1. Navigate to the launch directory:
   ```bash
   cd /root/ros2_humble_ws/src/ros2_m1_sim/launch/
   ```

2. Create a new launch file:
   ```bash
   nano simulation.launch.py
   ```

3. Add the following code:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for the simulation."""
    # Declare launch arguments
    use_ascii_viz_arg = DeclareLaunchArgument(
        'use_ascii_viz',
        default_value='true',
        description='Whether to use ASCII visualization'
    )
    
    use_turtlesim_viz_arg = DeclareLaunchArgument(
        'use_turtlesim_viz',
        default_value='false',
        description='Whether to use TurtleSim visualization'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_m1_sim'),
            'config',
            'simulation_params.yaml'
        ]),
        description='Path to the parameters file'
    )

    # Get launch configurations
    use_ascii_viz = LaunchConfiguration('use_ascii_viz')
    use_turtlesim_viz = LaunchConfiguration('use_turtlesim_viz')
    params_file = LaunchConfiguration('params_file')

    # Define nodes
    nodes = [
        # Simulation node
        Node(
            package='ros2_m1_sim',
            executable='simple_simulation',
            name='simple_simulation',
            output='screen',
            parameters=[params_file],
        ),
        
        # Subscriber node
        Node(
            package='ros2_m1_sim',
            executable='robot_subscriber',
            name='robot_subscriber',
            output='screen',
            parameters=[params_file],
        ),
    ]
    
    # Conditionally add ASCII visualizer
    ascii_viz_node = Node(
        package='ros2_m1_sim',
        executable='ascii_visualizer',
        name='ascii_visualizer',
        output='screen',
        condition=launch.conditions.IfCondition(use_ascii_viz),
    )
    nodes.append(ascii_viz_node)
    
    # Conditionally add TurtleSim nodes
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen',
        condition=launch.conditions.IfCondition(use_turtlesim_viz),
    )
    
    turtlesim_bridge_node = Node(
        package='ros2_m1_sim',
        executable='turtlesim_bridge',
        name='turtlesim_bridge',
        output='screen',
        condition=launch.conditions.IfCondition(use_turtlesim_viz),
    )
    
    nodes.extend([turtlesim_node, turtlesim_bridge_node])
    
    # Return launch description
    return LaunchDescription([
        use_ascii_viz_arg,
        use_turtlesim_viz_arg,
        params_file_arg,
        *nodes,
    ])
