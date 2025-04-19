#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class MoveInCircleNode(Node):
    def __init__(self):
        super().__init__('move_in_circle_node')

        # Declare parameters
        self.declare_parameter('diameter', 1.0)
        self.declare_parameter('namespace', '')

        self.diameter = self.get_parameter('diameter').get_parameter_value().double_value
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        # Compute radius and circle completion time
        self.radius = self.diameter / 2.0
        self.linear_speed = 0.2  # m/s
        self.angular_speed = self.linear_speed / self.radius  # rad/s

        # ROS 2 publisher
        self.cmd_vel_topic = self.resolve_topic_name('cmd_vel')
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Timer to publish velocity
        self.timer = self.create_timer(0.1, self.publish_cmd)

        # Logging
        self.get_logger().info(f"[{self.namespace}] Moving in circle of diameter {self.diameter:.2f} m")
        self.get_logger().info(f"[{self.namespace}] Publishing Twist to: {self.cmd_vel_topic}")

    def resolve_topic_name(self, base_name: str):
        """Prefix topic with namespace if provided"""
        if self.namespace:
            return f'/{self.namespace}/{base_name}'
        return f'/{base_name}'

    def publish_cmd(self):
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = self.angular_speed
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MoveInCircleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
