#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import math
import time


class RobotPublisher(Node):
    """
    Node that publishes simulated robot data.
    
    This node publishes:
    - Twist messages on cmd_vel topic
    - Odometry messages on odom topic
    """

    def __init__(self):
        super().__init__('robot_publisher')
        
        # Create publishers
        self.twist_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        
        # Create timer for publishing
        self.timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Initialize variables
        self.start_time = time.time()
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation = 0.0  # radians
        
        self.get_logger().info('Robot publisher node initialized')

    def timer_callback(self):
        """Callback function for the timer."""
        # Current time in seconds since start
        elapsed_time = time.time() - self.start_time
        
        # Create a simple circular motion pattern
        linear_velocity = 0.5  # m/s
        angular_velocity = 0.2  # rad/s
        
        # Update position and orientation
        self.orientation = (elapsed_time * angular_velocity) % (2 * math.pi)
        self.position_x = 2.0 * math.cos(self.orientation)
        self.position_y = 2.0 * math.sin(self.orientation)
        
        # Publish velocity command
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.twist_publisher.publish(twist_msg)
        
        # Publish odometry message
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
        odom_msg.twist.twist = twist_msg
        
        # Publish odometry
        self.odom_publisher.publish(odom_msg)
        
        self.get_logger().debug(f'Published robot data at position: ({self.position_x:.2f}, {self.position_y:.2f}), orientation: {self.orientation:.2f} rad')


def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)
    robot_publisher = RobotPublisher()
    
    try:
        rclpy.spin(robot_publisher)
    except KeyboardInterrupt:
        robot_publisher.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        # Clean up
        robot_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
