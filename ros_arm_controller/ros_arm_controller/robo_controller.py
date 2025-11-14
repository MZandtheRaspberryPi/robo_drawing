import math
import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import numpy as np
import matplotlib.pyplot as plt


from ros_sim.constants import DOF_NAMES, JOINT_REST_ANGLES
from ros_arm_controller.trajectory import TrajectoryPlanner
from ros_arm_controller.constants import (
    CACHE_DIR,
    DEFAULT_EPS_XYZ,
    DEFAULT_EPS_ORI,
    JOINT_ANGLE_DIFF_TOL,
    X_PLOT_LIM,
    Y_PLOT_LIM,
    Z_PLOT_LIM,
)


class RoboController(Node):

    def __init__(self):
        super().__init__("robo_controller")

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_sub_cb, 5
        )
        self.joint_state_pub = self.create_publisher(
            JointState, "/target_joint_states", 5
        )

        self.use_clock_time = False
        # self.declare_parameter('target_frame', rclpy.Parameter.Type.STRING)

        self.traj_planner = None  # need starting angles
        self.cur_joint_angles = None
        self.cur_time = 0.0
        self.target_loop_rate = 10
        self.target_dt = 1 / self.target_loop_rate

        self.timer = self.create_timer(self.target_dt, self.timer_callback)

        self.get_logger().info(
            f"using sim time: {self.get_parameter('use_sim_time').get_parameter_value().bool_value}"
        )

        self.starting_time_sec = None
        self.time_updated = False
        self.xyz_plotted = []
        self.z_threshold = 0.5

        self.xlim = X_PLOT_LIM
        self.ylim = Y_PLOT_LIM
        self.zlim = Z_PLOT_LIM

        self.traj_seg_name = None

    def joint_sub_cb(self, msg):

        joint_names = msg.name
        joint_positions = msg.position
        self.cur_joint_angles = joint_positions

        if self.traj_planner is None:
            self.get_logger().info(f"initializing trajectory planner")
            traj_planner = TrajectoryPlanner(loop_rate=self.target_loop_rate)
            self.get_logger().info(f"calculating traj")
            print(f"start: {joint_positions}")
            success = traj_planner.make_joint_traj(cur_angles=joint_positions)
            if not success:
                self.get_logger().error(
                    f"at least one waypoint didn't solve for convergence succesfully..."
                )
            self.get_logger().info(f"done calculating traj")
            self.traj_planner = traj_planner

    def timer_callback(self):

        if self.traj_planner is None or self.cur_joint_angles is None:
            return

        # get updated time if using sim time...
        if self.starting_time_sec is None and not self.time_updated:
            self.time_updated = True
            return
        elif self.starting_time_sec is None and self.time_updated:
            if self.use_clock_time:
                self.starting_time_sec = self.get_clock().now().nanoseconds * 1e-9
            else:
                self.starting_time_sec = 0.0
                self.cur_time = 0.0
            print(f"starting time: {self.starting_time_sec}")

        cur_angles = self.cur_joint_angles

        xyz_cur, euler_cur = self.traj_planner.get_transform_last_frame(cur_angles)

        if xyz_cur[2] < self.z_threshold:
            self.xyz_plotted.append(xyz_cur)

        if self.use_clock_time:
            cur_time = self.get_clock().now().nanoseconds * 1e-9
        else:
            cur_time = self.cur_time
        time_since_start = cur_time - self.starting_time_sec

        name, joint_angles, xyz, ori, done = self.traj_planner.get_joint_angles(
            time_since_start
        )

        if name != self.traj_seg_name:
            self.get_logger().info(f"doing: {name}")
            self.traj_seg_name = name

        joint_angle_diff = np.linalg.norm(np.array(cur_angles) - np.array(joint_angles))

        ori_diff = np.linalg.norm(euler_cur - np.array(ori))

        if not self.use_clock_time:
            if joint_angle_diff < JOINT_ANGLE_DIFF_TOL:
                self.cur_time += self.target_dt
            else:
                self.get_logger().info(
                    f"joint_angle_diff: {joint_angle_diff}, waiting..."
                )
        self.get_logger().info(
            f"time_since_start: {time_since_start}, name: {name} done: {done}, cur xyz: {xyz_cur}, cur ori: {euler_cur}, angles_diff: {ori_diff} cur angles {cur_angles} goal angles: {joint_angles}, goal xyz: {xyz} goal ori: {ori}"
        )

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()

        if done:
            joint_angles = JOINT_REST_ANGLES
            self.traj_planner = None
            self.starting_time_sec = None
            self.time_updated = False

            fig = plt.figure()
            ax = fig.add_subplot(projection="3d")

            arr = np.array(self.xyz_plotted)

            x = arr[:, 0] if len(arr) > 1 else []
            y = arr[:, 1] if len(arr) > 1 else []
            z = arr[:, 2] if len(arr) > 1 else []
            ax.plot(x, y, z, label="trajectory")
            ax.set_xlabel("x")
            ax.set_ylabel("y")
            ax.set_zlabel("z")
            ax.set_xlim(*self.xlim)
            ax.set_ylim(*self.ylim)
            ax.set_zlim(*self.zlim)

            # rotation around z, degrees
            ax.azim = 180
            # distance from center point
            ax.dist = 0.15
            # angle between eye and xy plane
            ax.elev = 45

            fig.savefig(os.path.join(CACHE_DIR, "traj_act.png"))

            self.xyz_plotted.clear()

        for i in range(len(joint_angles)):
            name = DOF_NAMES[i]
            pos = joint_angles[i]
            joint_msg.position.append(pos)
            joint_msg.name.append(name)
        self.joint_state_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)

    ctrl = RoboController()
    rclpy.spin(ctrl)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ctrl.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
