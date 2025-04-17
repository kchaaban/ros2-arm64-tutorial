#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import math
import time
import numpy as np


class SimpleSimulation(Node):
    """
    Node that simulates a 2D differential drive robot.
    
    This simulation:
    - Subscribes to cmd_vel for velocity commands
    - Publishes robot state on odom topic
    - Broadcasts TF transforms between frames
    - Simulates physics including inertia and friction
    """

    def __init__(self):
        super().__init__('simple_simulation')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.1)  # meters
        self.declare_parameter('wheel_separation', 0.5)  # meters
        self.declare_parameter('max_velocity', 1.0)  # m/s
        self.declare_parameter('max_acceleration', 0.5)  # m/s^2
        self.declare_parameter('friction', 0.1)  # damping factor
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.friction = self.get_parameter('friction').value
        
        # Create publisher
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        
        # Create subscriber
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create simulation timer
        self.sim_timer_period = 0.01  # seconds (100 Hz)
        self.sim_timer = self.create_timer(self.sim_timer_period, self.simulation_step)
        
        # Initialize robot state
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation = 0.0  # radians
        self.linear_velocity = 0.0  # m/s
        self.angular_velocity = 0.0  # rad/s
        self.target_linear_velocity = 0.0  # m/s
        self.target_angular_velocity = 0.0  # rad/s
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Simple simulation node initialized with the following parameters:')
        self.get_logger().info(f'  - Wheel radius: {self.wheel_radius} m')
        self.get_logger().info(f'  - Wheel separation: {self.wheel_separation} m')
        self.get_logger().info(f'  - Max velocity: {self.max_velocity} m/s')
        self.get_logger().info(f'  - Max acceleration: {self.max_acceleration} m/s^2')
        self.get_logger().info(f'  - Friction: {self.friction}')

    def cmd_vel_callback(self, msg):
        """Callback function for velocity commands."""
        # Update target velocities
        self.target_linear_velocity = min(msg.linear.x, self.max_velocity)
        self.target_angular_velocity = msg.angular.z
        
        self.get_logger().debug(
            f'Received velocity command: linear={self.target_linear_velocity:.2f} m/s, '
            f'angular={self.target_angular_velocity:.2f} rad/s'
        )

    def simulation_step(self):
        """Perform one step of the simulation."""
        # Calculate time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time
        
        # Apply acceleration limits
        linear_velocity_error = self.target_linear_velocity - self.linear_velocity
        angular_velocity_error = self.target_angular_velocity - self.angular_velocity
        
        linear_acceleration = np.clip(
            linear_velocity_error / dt, 
            -self.max_acceleration, 
            self.max_acceleration
        )
        
        # Update velocities
        self.linear_velocity += linear_acceleration * dt
        self.angular_velocity += angular_velocity_error * dt
        
        # Apply friction
        self.linear_velocity *= (1.0 - self.friction * dt)
        self.angular_velocity *= (1.0 - self.friction * dt)
        
        # Update position and orientation
        self.position_x += self.linear_velocity * math.cos(self.orientation) * dt
        self.position_y += self.linear_velocity * math.sin(self.orientation) * dt
        self.orientation += self.angular_velocity * dt
        
        # Normalize orientation to [-pi, pi]
        self.orientation = (self.orientation + math.pi) % (2 * math.pi) - math.pi
        
        # Create and publish odometry message
        self.publish_odometry()
        
        # Broadcast TF transform
        self.broadcast_transform()

    def publish_odometry(self):
        """Publish odometry message."""
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose = Pose()
        odom_msg.pose.pose.position = Point(x=float(self.position_x), 
                                           y=float(self.position_y), 
                                           z=0.0)
        
        # Set orientation as quaternion
        qx = 0.0
        qy = 0.0
        qz = math.sin(self.orientation / 2)
        qw = math.cos(self.orientation / 2)
        odom_msg.pose.pose.orientation = Quaternion(x=float(qx), 
                                                   y=float(qy), 
                                                   z=float(qz), 
                                                   w=float(qw))
        
        # Set velocity
        odom_msg.twist.twist = Twist()
        odom_msg.twist.twist.linear.x = float(self.linear_velocity)
        odom_msg.twist.twist.angular.z = float(self.angular_velocity)
        
        # Publish odometry
        self.odom_publisher.publish(odom_msg)

    def broadcast_transform(self):
        """Broadcast transform from odom to base_link."""
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        
        # Set translation
        tf_msg.transform.translation.x = float(self.position_x)
        tf_msg.transform.translation.y = float(self.position_y)
        tf_msg.transform.translation.z = 0.0
        
        # Set rotation
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = float(math.sin(self.orientation / 2))
        tf_msg.transform.rotation.w = float(math.cos(self.orientation / 2))
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)
    simple_simulation = SimpleSimulation()
    
    try:
        rclpy.spin(simple_simulation)
    except KeyboardInterrupt:
        simple_simulation.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        # Clean up
        simple_simulation.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
