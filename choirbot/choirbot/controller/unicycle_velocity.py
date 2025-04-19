from rclpy.node import Node
from .. import Pose
from .controller import Controller
from geometry_msgs.msg import Vector3, Twist
import numpy as np

class UnicycleVelocityController(Controller):
    def __init__(self, pose_handler: str=None, pose_topic: str=None):
        super().__init__(pose_handler, pose_topic)
        self.subscription = self.create_subscription(Vector3, 'velocity', self.control_callback, 1)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.yaw = 0.0
        self.yaw_old = 0.0
        self.yaw_old_old = 0.0

    def control_callback(self, msg):
        if self.current_pose.position is None:
            return

        # Angular override from guidance (used during yaw alignment)
        if abs(msg.z) > 0.01 and abs(msg.x) < 1e-3 and abs(msg.y) < 1e-3:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = msg.z
            self.publisher_.publish(cmd)
            return

        # Otherwise: use full XY guidance
        u_global = np.array([msg.x, msg.y])
        self.get_yaw()
        cos_yaw = np.cos(self.yaw)
        sin_yaw = np.sin(self.yaw)

        # Convert global to local frame
        u_local_x = cos_yaw * u_global[0] + sin_yaw * u_global[1]
        u_local_y = -sin_yaw * u_global[0] + cos_yaw * u_global[1]

        # Compute Twist command
        v = np.clip(u_local_x, -0.3, 0.3)
        w = np.clip(np.arctan2(u_local_y, u_local_x), -1.0, 1.0)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.publisher_.publish(cmd)

    def get_yaw(self):
        quat = np.copy(self.current_pose.orientation)
        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]
        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw_new = np.arctan2(siny_cosp, cosy_cosp)
        yaw_array = np.array([self.yaw_old_old, self.yaw_old, yaw_new])
        yaw_array_new = np.unwrap(yaw_array)

        # shift value of variables
        self.yaw, self.yaw_old, self.yaw_old_old = yaw_array_new[2], self.yaw, self.yaw_old

    def send_input(self, u):
        msg = Twist()
        msg.linear.x = u[0]
        msg.angular.z = u[1]
        self.publisher_.publish(msg)
