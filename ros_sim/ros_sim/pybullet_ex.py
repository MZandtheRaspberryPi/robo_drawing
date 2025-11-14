import pybullet as pb
import pybullet_data
from pybullet_utils import bullet_client
import numpy as np

import math
import os

import rclpy
from rclpy.node import Node


from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from ros_sim.constants import MAX_FORCE, DOF_NAMES, JOINT_REST_ANGLES, LINK_NAME
from ros_sim.utils import get_urdf_path
from ros_sim.pybullet_sim import PybulletSimParams, PybulletSim

import time


class RosSim(Node):

    def __init__(self):
        super().__init__("ros_sim_node")

        self.clock_pub = self.create_publisher(Clock, "/clock", 1)
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 5)

        self.joint_state_sub = self.create_subscription(
            JointState, "/target_joint_states", self.joint_target_cb, 5
        )

        self.cur_time = 0.0

        self.run_demo = True
        self.path_to_urdf = get_urdf_path()

        self.get_logger().info(f"loading: {self.path_to_urdf}")

        self.params = PybulletSimParams(
            urdf_file_path=self.path_to_urdf, dof_names=DOF_NAMES, n_dof=len(DOF_NAMES)
        )

        self.sim = PybulletSim(self.params)
        self.timer = self.create_timer(self.params.dt, self.timer_callback)
        self.sim.setup()
        self.goal_angles = JOINT_REST_ANGLES

    def joint_target_cb(self, msg: JointState):
        """Cache the target joint angles from a controller

        Args:
            msg (JointState): ros joint state message
        """
        goal_angles = [None] * self.params.n_dof

        for i in range(len(msg.position)):

            joint_name = msg.name[i]
            joint_idx = DOF_NAMES.index(joint_name)
            goal_angles[joint_idx] = msg.position[i]

        self.goal_angles = goal_angles

    def publish_time(self):
        """Publish the current time according to the simulator"""
        sec = math.floor(self.cur_time)
        nanosec = math.floor((self.cur_time - sec) * 1e9)
        msg = Clock()
        msg.clock.sec = sec
        msg.clock.nanosec = nanosec
        self.clock_pub.publish(msg)

    def timer_callback(self):
        """Regularly check the goal angles and step the simulation accordingly, then publish the new simulation time."""

        print(f"goals: {self.goal_angles}")
        controls = self.goal_angles

        self.sim.step(controls)

        self.cur_time += self.params.dt

        sec = math.floor(self.cur_time)
        nanosec = math.floor((self.cur_time - sec) * 1e9)
        joint_msg = JointState()
        joint_msg.header.stamp.sec = sec
        joint_msg.header.stamp.nanosec = nanosec

        pos, vel, tau = self.sim.get_joint_pos_vel()

        print(f"cur pos: {pos}")

        for i in range(len(pos)):
            name = self.params.dof_names[i]
            joint_msg.position.append(pos[i])
            joint_msg.velocity.append(vel[i])
            joint_msg.effort.append(tau[i])
            joint_msg.name.append(name)
        self.publish_time()
        self.joint_state_pub.publish(joint_msg)

        link_world_pos, link_world_ori, link_lin_vel, link_ang_vel = (
            self.sim.get_link_state(LINK_NAME)
        )

        if np.any(np.abs(tau) == MAX_FORCE):
            self.get_logger().error(
                f"force is at max, please check for collisions. Max force: {MAX_FORCE}, joint torques: {tau}"
            )

    def shutdown(self):
        """Close down."""
        self.sim.close()


def main(args=None):
    rclpy.init(args=args)

    sim = RosSim()

    rclpy.spin(sim)

    sim.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
