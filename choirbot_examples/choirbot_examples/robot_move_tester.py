#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MoveForwardNode(Node):
    def __init__(self):
        super().__init__('move_forward_node')

        # Get distance parameter (passed via launch)
        self.declare_parameter('distance', 1.0)
        self.target_distance = self.get_parameter('distance').get_parameter_value().double_value

        # Namespace-aware topic resolution
        self.odom_topic = self.resolve_topic_name('odom')
        self.cmd_vel_topic = self.resolve_topic_name('cmd_vel')

        # ROS 2 pub/sub
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Internal state
        self.initial_pose_recorded = False
        self.start_x = 0.0
        self.current_x = 0.0
        self.moving = True
        self.speed = 0.2  # m/s

        # Periodic command publishing
        self.timer = self.create_timer(0.1, self.publish_cmd)

        # Log status
        self.get_logger().info(f"[{self.get_namespace()}] Target distance: {self.target_distance:.2f} meters")
        self.get_logger().info(f"[{self.get_namespace()}] Subscribing to: {self.odom_topic}")
        self.get_logger().info(f"[{self.get_namespace()}] Publishing to: {self.cmd_vel_topic}")

    def resolve_topic_name(self, base_name: str):
        """Auto-prefix topics with namespace, if not root ('/')"""
        ns = self.get_namespace()
        if ns == '/':
            return f'/{base_name}'
        return f'{ns}/{base_name}'

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x

        if not self.initial_pose_recorded:
            self.start_x = self.current_x
            self.initial_pose_recorded = True
            self.get_logger().info(f"[{self.get_namespace()}] Starting X: {self.start_x:.2f}")

        distance_moved = self.current_x - self.start_x
        if distance_moved >= self.target_distance:
            self.moving = False
            self.get_logger().info(f"[{self.get_namespace()}] Moved {distance_moved:.2f}m. Stopping.")

    def publish_cmd(self):
        cmd = Twist()
        cmd.linear.x = self.speed if self.moving else 0.0
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MoveForwardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
