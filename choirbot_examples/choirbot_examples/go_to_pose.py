#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw in radians"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class GoToPose(Node):
    def __init__(self):
        super().__init__('go_to_pose_controller')

        self.declare_parameter("namespace", "")
        self.declare_parameter("goal_position", [0.0, 0.0, 0.0])
        self.declare_parameter("goal_orientation", [0.0, 0.0, 0.0, 1.0])

        ns = self.get_parameter("namespace").value.strip('/')
        self.ns_prefix = f'/{ns}' if ns else ''

        goal_position = self.get_parameter("goal_position").value
        goal_orientation = self.get_parameter("goal_orientation").value

        self.goal_x = goal_position[0]
        self.goal_y = goal_position[1]
        self.goal_theta = quaternion_to_yaw(*goal_orientation)

        self.pose_received = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Previous twist values for low-pass filtering
        self.prev_linear_x = 0.0
        self.prev_angular_z = 0.0

        self.vel_pub = self.create_publisher(Twist, f'{self.ns_prefix}/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, f'{self.ns_prefix}/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.get_logger().info(f"[{ns}] Going to goal: x={self.goal_x:.3f}, y={self.goal_y:.3f}, yaw={math.degrees(self.goal_theta):.2f}°")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.pose_received = True

    def smooth_command(self, new, prev, alpha=0.6):
        """Low-pass filter for velocity smoothing"""
        return alpha * prev + (1 - alpha) * new

    def control_loop(self):
        if not self.pose_received:
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.hypot(dx, dy)

        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.current_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # normalize

        twist = Twist()

        # Movement logic
        if distance > 0.05:
            if abs(angle_error) > 0.1:
                # Turn in place
                twist.linear.x = 0.0
                twist.angular.z = max(min(0.4 * angle_error, 0.4), -0.4)
            else:
                # Move forward and steer
                linear_gain = 0.3 if distance > 0.5 else 0.1
                angular_gain = 0.5 if distance > 0.5 else 0.3
                twist.linear.x = min(0.2, linear_gain * distance)
                twist.angular.z = max(min(angular_gain * angle_error, 0.4), -0.4)
        else:
            # Reached position, now align heading
            # heading_error = self.goal_theta - self.current_theta
            # heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
            # if abs(heading_error) > 0.05:
            #     twist.linear.x = 0.0
            #     twist.angular.z = max(min(0.3 * heading_error, 0.3), -0.3)
            # else:
            #     self.get_logger().info(f"[{self.ns_prefix}] Goal Reached!")
            #     twist.linear.x = 0.0
            #     twist.angular.z = 0.0
            heading_error = self.goal_theta - self.current_theta
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

            if abs(heading_error) > 0.01:
                twist.linear.x = 0.0
                twist.angular.z = max(min(0.3 * heading_error, 0.3), -0.3)
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                if not hasattr(self, "goal_logged"):
                    self.get_logger().info(f"[{self.ns_prefix}] Goal Reached! Final heading aligned.")
                    self.goal_logged = True


        # Smooth final command
        twist.linear.x = self.smooth_command(twist.linear.x, self.prev_linear_x)
        twist.angular.z = self.smooth_command(twist.angular.z, self.prev_angular_z)
        self.prev_linear_x = twist.linear.x
        self.prev_angular_z = twist.angular.z

        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GoToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
