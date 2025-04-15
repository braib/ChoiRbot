#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

class JoystickTeleop(Node):
    def __init__(self, namespace=''):
        super().__init__('joystick_teleop')

        self.namespace = namespace.strip('/')
        topic_name = f'/{self.namespace}/cmd_vel' if self.namespace else '/cmd_vel'

        self.publisher = self.create_publisher(Twist, topic_name, 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.linear_axis = 1   # Typically left stick vertical
        self.angular_axis = 0  # Typically left stick horizontal
        self.linear_scale = 0.5
        self.angular_scale = 1.0
        self.enable_button = 5


        self.get_logger().info(f"Publishing to: {topic_name}")

    def joy_callback(self, msg):
        # Only send velocity if enable button is pressed
        if len(msg.buttons) > self.enable_button and msg.buttons[self.enable_button] == 1:
            twist = Twist()
            twist.linear.x = self.linear_scale * msg.axes[self.linear_axis]
            twist.angular.z = self.angular_scale * msg.axes[self.angular_axis]
            self.publisher.publish(twist)
        # else: do nothing (don't publish)


def main(args=None):
    rclpy.init(args=args)

    import sys
    ns = ''
    for arg in sys.argv:
        if arg.startswith("namespace:="):
            ns = arg.split(":=")[1]

    joystick_node = JoystickTeleop(namespace=ns)
    rclpy.spin(joystick_node)
    joystick_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
