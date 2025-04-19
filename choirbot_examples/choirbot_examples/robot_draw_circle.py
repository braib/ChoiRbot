#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveInCircleNode(Node):
    def __init__(self):
        super().__init__('move_in_circle_node')

        # Declare parameter early
        # self.declare_parameter('diameter', 1.0)
        # self.diameter = self.get_parameter('diameter').get_parameter_value().double_value

        #  Define namespace safely using rclpy API
        # self._namespace = self.get_namespace().strip('/')
        
        self.diameter = 1.0
        # Compute circle motion
        self.radius = self.diameter / 2.0
        self.linear_speed = 0.2
        self.angular_speed = self.linear_speed / self.radius

        # Publisher
        # self.cmd_vel_topic = self.resolve_topic_name('cmd_vel')
        self.cmd_vel_topic = '/agent_0/cmd_vel'
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Timer
        self.timer = self.create_timer(0.1, self.publish_cmd)

        # self.get_logger().info(f"[{self._namespace}] Moving in circle of diameter {self.diameter:.2f} m")
        # self.get_logger().info(f"[{self._namespace}] Publishing to: {self.cmd_vel_topic}")

    # def resolve_topic_name(self, base_name: str):
    #     if self._namespace:
    #         return f'/{self._namespace}/{base_name}'
    #     return f'/{base_name}'

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
