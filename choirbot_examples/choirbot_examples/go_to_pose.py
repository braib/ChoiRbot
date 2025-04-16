#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math

def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw in radians"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class GoToPoseSensorFusion(Node):
    def __init__(self):
        super().__init__('go_to_pose_sensor_fusion_controller')

        # Parameters for namespace and goal
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

        # Robot state
        self.pose_received = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # IMU state
        self.imu_yaw = None
        self.last_imu_time = None

        # Publishers and subscribers with namespace
        self.vel_pub = self.create_publisher(Twist, f'{self.ns_prefix}/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, f'{self.ns_prefix}/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, f'{self.ns_prefix}/imu', self.imu_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.get_logger().info(f"[{ns}] Going to goal: x={self.goal_x:.3f}, y={self.goal_y:.3f}, yaw={math.degrees(self.goal_theta):.2f}°")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.pose_received = True

        # Optional: Initialize IMU yaw if not done yet
        if self.imu_yaw is None:
            self.imu_yaw = self.current_theta

    def imu_callback(self, msg):
        # Integrate angular velocity.z (gyroscope) over time to estimate yaw
        angular_z = msg.angular_velocity.z
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_imu_time is not None:
            dt = now - self.last_imu_time
            if dt > 0 and abs(angular_z) < 10:  # Discard outliers
                self.imu_yaw += angular_z * dt
                self.imu_yaw = math.atan2(math.sin(self.imu_yaw), math.cos(self.imu_yaw))  # normalize
        self.last_imu_time = now

    def control_loop(self):
        if not self.pose_received:
            return

        # Sensor Fusion: Complementary filter
        if self.imu_yaw is not None:
            # Complementary filter: mostly odom, little IMU
            fused_theta = 0.98 * self.current_theta + 0.02 * self.imu_yaw
            fused_theta = math.atan2(math.sin(fused_theta), math.cos(fused_theta))
        else:
            fused_theta = self.current_theta

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - fused_theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # normalize

        twist = Twist()

        if distance > 0.05:
            if abs(angle_error) > 0.1:
                twist.angular.z = 1.0 * angle_error
            else:
                twist.linear.x = 0.5 * distance
                twist.angular.z = 0.5 * angle_error
        else:
            heading_error = self.goal_theta - fused_theta
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
            if abs(heading_error) > 0.05:
                twist.angular.z = 0.5 * heading_error
            else:
                self.get_logger().info(f"[{self.ns_prefix}] Goal Reached!")
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseSensorFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
