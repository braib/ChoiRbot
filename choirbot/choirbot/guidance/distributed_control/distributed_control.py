from geometry_msgs.msg import Vector3
from ..guidance import Guidance
import numpy as np
from time import sleep
from time import time


class DistributedControlGuidance(Guidance):

    # def __init__(self, update_frequency: float, pose_handler: str=None, pose_topic: str=None, input_topic: str = 'velocity'):
    #     super().__init__(pose_handler, pose_topic)
    #     self.publisher_ = self.create_publisher(Vector3, input_topic, 1)
    #     self.update_frequency = update_frequency
    #     self.timer = self.create_timer(1.0/self.update_frequency, self.control)
    #     self.get_logger().info('Guidance {} started'.format(self.agent_id))

    #     self.formation_achieved = False
    #     self.transition_done = False
    #     self.goal_center = np.array([2.0, 2.0, 0.0])  # your desired (x, y)
    #     self.offset_in_formation = self.get_parameter_or('formation_offset', np.zeros(3))
    #     self.t_start_move = None

    def __init__(self, update_frequency: float, pose_handler: str=None, pose_topic: str=None, input_topic: str = 'velocity'):
        super().__init__(pose_handler, pose_topic)
        self.publisher_ = self.create_publisher(Vector3, input_topic, 1)
        self.update_frequency = update_frequency
        self.timer = self.create_timer(1.0/self.update_frequency, self.control)
        self.get_logger().info('Guidance {} started'.format(self.agent_id))

        self.formation_achieved = False
        self.transition_done = False

        # Load goal_center from ROS parameter
        from rclpy.parameter import Parameter
        goal_center_param = self.get_parameter_or('goal_center', [2.0, 2.0, 0.0])
        if isinstance(goal_center_param, Parameter):
            goal_center_param = goal_center_param.value
        self.goal_center = np.array(goal_center_param)
        self.get_logger().info(f"Goal center loaded: {self.goal_center}")

        self.offset_in_formation = self.get_parameter_or('formation_offset', np.zeros(3))
        self.t_start_move = None


    # def control(self):
    #     # skip if position is not available yet
    #     if self.current_pose.position is None:
    #         return
        
    #     # exchange current position with neighbors
    #     data = self.communicator.neighbors_exchange(self.current_pose, self.in_neighbors, self.out_neighbors, False)

    #     # compute input
    #     u = self.evaluate_input(data)

    #     # send input to planner/controller
    #     self.send_input(u)



    def control(self):
        if self.current_pose.position is None:
            return

        data = self.communicator.neighbors_exchange(
            self.current_pose, self.in_neighbors, self.out_neighbors, False
        )

        if not hasattr(self, 'formation_achieved'):
            self.formation_achieved = False
        if not hasattr(self, 'transition_started'):
            self.transition_started = False

        if not self.formation_achieved:
            u = self.evaluate_input(data)
            if self.formation_achieved:
                self.get_logger().info("Formation achieved! Locking and preparing for translation...")
                self.triangle_center = self.compute_triangle_center(data)
                self.offset_from_center = self.current_pose.position - self.triangle_center
            self.send_input(u)
            return

        if not self.transition_started:
            self.transition_started = True
            self.get_logger().info("Now translating the whole triangle to new center...")
            self.goal_center = np.array([5.0, 0.0, 0.0])  # new desired triangle center

        target = self.goal_center + self.offset_from_center
        error = target - self.current_pose.position

        # Stop if close
        if np.linalg.norm(error) < 0.05:
            u = np.zeros(3)
            self.get_logger().info("Reached translated formation target.")
        else:
            u = 0.2 * error

        self.send_input(u)
    
    def compute_triangle_center(self, neigh_data):
        positions = [self.current_pose.position] + [pose.position for pose in neigh_data.values()]
        return sum(positions) / len(positions)





    def send_input(self, u):
        msg = Vector3()

        msg.x = u[0]
        msg.y = u[1]
        msg.z = u[2]

        self.publisher_.publish(msg)

    def evaluate_input(self, neigh_data):
        raise NotImplementedError

