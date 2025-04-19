from geometry_msgs.msg import Vector3
from ..guidance import Guidance
import numpy as np
from time import time
from rclpy.parameter import Parameter

class DistributedControlGuidance(Guidance):
    def __init__(self, update_frequency: float, pose_handler: str = None, pose_topic: str = None, input_topic: str = 'velocity'):
        super().__init__(pose_handler, pose_topic)
        self.publisher_ = self.create_publisher(Vector3, input_topic, 1)
        self.update_frequency = update_frequency
        self.timer = self.create_timer(1.0 / self.update_frequency, self.control)
        self.get_logger().info(f'Guidance {self.agent_id} started')

        # State flags
        self.formation_achieved = False
        self.rotation_complete = False
        self.goal_center_reached = False
        self.offset_initialized = False

        # Parameters
        goal_param = self.get_parameter_or('goal_center', [5.0, 0.0, 0.0])
        if isinstance(goal_param, Parameter):
            goal_param = goal_param.value
        self.goal_center = np.array(goal_param)

        self.yaw = 0.0
        self.desired_yaw = 0.0
        self.t_align_start = None
        self.align_timeout = 5.0

    def control(self):
        if self.current_pose.position is None:
            return

        data = self.communicator.neighbors_exchange(self.current_pose, self.in_neighbors, self.out_neighbors, False)

        # Phase 1: Formation
        u_formation = self.evaluate_input(data)
        if not self.formation_achieved:
            self.send_input(u_formation)
            return

        # Phase 2: Initialize once after formation
        if not self.offset_initialized:
            self.offset_initialized = True
            self.triangle_start = self.compute_triangle_center(data)
            self.offset_from_center = self.current_pose.position - self.triangle_start
            direction = self.goal_center - self.triangle_start
            self.desired_yaw = np.arctan2(direction[1], direction[0])
            self.t_align_start = time()
            self.get_logger().info(f"[{self.agent_id}] Formation done. Aligning to yaw: {self.desired_yaw:.2f}")
            return

        # Phase 3: Yaw alignment
        if not self.rotation_complete:
            self.get_yaw()
            yaw_error = np.arctan2(np.sin(self.desired_yaw - self.yaw), np.cos(self.desired_yaw - self.yaw))

            if time() - self.t_align_start > self.align_timeout:
                self.rotation_complete = True
                self.get_logger().warn(f"[{self.agent_id}] Yaw timeout. Proceeding anyway.")
                return

            if abs(yaw_error) > 0.05:
                self.send_input(np.array([0.0, 0.0, np.clip(1.0 * yaw_error, -1.0, 1.0)]))
                return
            else:
                self.rotation_complete = True
                self.get_logger().info(f"[{self.agent_id}] Yaw aligned.")
                return

        # Phase 4: Move toward goal center while maintaining formation
        current_center = self.compute_triangle_center(data)
        remaining = self.goal_center - current_center

        if np.linalg.norm(remaining) < 0.05:
            self.send_input(np.zeros(3))
            self.get_logger().info(f"[{self.agent_id}] Reached goal center.")
            return

        # Maintain formation + move together
        u_formation = self.evaluate_input(data)
        v = np.clip(np.linalg.norm(remaining), 0.05, 0.2)
        direction = remaining / np.linalg.norm(remaining)
        u_translate = direction * v

        u = np.zeros(3)
        u[:2] = u_formation[:2] + u_translate[:2]
        self.send_input(u)

    def evaluate_input(self, neigh_data):
        return NotImplementedError

    def get_yaw(self):
        quat = self.current_pose.orientation
        x, y, z, w = quat[0], quat[1], quat[2], quat[3]
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.yaw = np.arctan2(siny_cosp, cosy_cosp)

    def compute_triangle_center(self, neigh_data):
        positions = [self.current_pose.position] + [pose.position for pose in neigh_data.values()]
        return sum(positions) / len(positions)

    def send_input(self, u):
        msg = Vector3()
        msg.x = u[0]
        msg.y = u[1]
        msg.z = u[2]
        self.publisher_.publish(msg)
