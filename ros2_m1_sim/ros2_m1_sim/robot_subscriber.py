#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class RobotSubscriber(Node):
    """
    Node that subscribes to robot data.
    
    This node subscribes to:
    - cmd_vel topic for robot velocity
    - odom topic for robot odometry
    """

    def __init__(self):
        super().__init__('robot_subscriber')
        
        # Create subscribers
        self.twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10)
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        # Initialize variables
        self.last_position = None
        self.distance_traveled = 0.0
        
        self.get_logger().info('Robot subscriber node initialized')

    def twist_callback(self, msg):
        """Callback function for cmd_vel messages."""
        linear_velocity = math.sqrt(
            msg.linear.x ** 2 + 
            msg.linear.y ** 2 + 
            msg.linear.z ** 2
        )
        angular_velocity = math.sqrt(
            msg.angular.x ** 2 + 
            msg.angular.y ** 2 + 
            msg.angular.z ** 2
        )
        
        self.get_logger().info(
            f'Robot velocity: linear={linear_velocity:.2f} m/s, angular={angular_velocity:.2f} rad/s'
        )

    def odom_callback(self, msg):
        """Callback function for odom messages."""
        # Get current position
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to Euler angles (yaw)
        # This is a simplified conversion that only extracts yaw
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Calculate distance traveled
        if self.last_position is not None:
            dx = position.x - self.last_position.x
            dy = position.y - self.last_position.y
            distance = math.sqrt(dx ** 2 + dy ** 2)
            self.distance_traveled += distance
        
        self.last_position = position
        
        # Log position and orientation
        self.get_logger().info(
            f'Robot position: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f}), ' +
            f'yaw: {yaw:.2f} rad, distance traveled: {self.distance_traveled:.2f} m'
        )


def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)
    robot_subscriber = RobotSubscriber()
    
    try:
        rclpy.spin(robot_subscriber)
    except KeyboardInterrupt:
        robot_subscriber.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        # Clean up
        robot_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
