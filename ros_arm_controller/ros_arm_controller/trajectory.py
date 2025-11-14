from bisect import bisect_left
import copy
from dataclasses import dataclass
import math
import pickle
from typing import Callable, Dict, Tuple, Optional

import numpy as np
from ros_arm_controller.constants import (
    CACHE_DIR,
    PAPER_HEIGHT,
    X_PLOT_LIM,
    Y_PLOT_LIM,
    Z_PLOT_LIM,
)
import os

from ament_index_python.packages import get_package_share_directory
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tqdm import tqdm
import matplotlib.pyplot as plt

from ros_sim.utils import get_urdf_path
from ros_sim.constants import DOF_NAMES, JOINT_REST_ANGLES, LINK_NAME
from ros_sim.pybullet_sim import PybulletSimParams, PybulletSim
import pybullet as pb


@dataclass
class TrajSeg:
    """This is a container to hold trajectory segments that will be called
    to get key position and orientations along the segment as a function of time.
    """

    name: str  # some name to identify the segment, like M start left line
    duration: float
    interpolate_joint_angles: bool
    fn_kwargs: (
        Dict  # key word arguments to pass to the function, like starting and ending x
    )
    # a function that will be called with time as an input to get out the XYZ position of the robot end effector in world coordinates
    # time, starting time, and duration are needed to be arguments, aside from that fn_kwargs are allowed to pass other arguments
    fn: Callable


def draw_line(
    t: float,
    starting_x: float,
    ending_x: float,
    starting_y: float,
    ending_y: float,
    starting_time: float,
    duration: float,
    starting_z: float,
    ending_z: float,
    starting_roll: float,
    ending_roll: float,
    starting_pitch: float,
    ending_pitch: float,
    starting_yaw: float,
    ending_yaw: float,
) -> Tuple[Tuple[float], Tuple[float]]:
    """Helper function to calculate position and orientation for a given time instant according to parameters

    Args:
        t (float): _description_
        starting_x (float): _description_
        ending_x (float): _description_
        starting_y (float): _description_
        ending_y (float): _description_
        starting_time (float): _description_
        duration (float): _description_
        starting_z (float): _description_
        ending_z (float): _description_
        starting_roll (float): _description_
        ending_roll (float): _description_
        starting_pitch (float): _description_
        ending_pitch (float): _description_
        starting_yaw (float): _description_
        ending_yaw (float): _description_

    Returns:
        Tuple[Tuple[float], Tuple[float]]: position 3 dimensions in meters, orientation 3 dimensions in euler angles radians
    """
    ending_time = starting_time + duration
    time_pct = (t - starting_time) / (ending_time - starting_time)

    cur_x = (ending_x - starting_x) * time_pct + starting_x
    cur_y = (ending_y - starting_y) * time_pct + starting_y
    cur_z = (ending_z - starting_z) * time_pct + starting_z

    cur_roll = (ending_roll - starting_roll) * time_pct + starting_roll
    cur_pitch = (ending_pitch - starting_pitch) * time_pct + starting_pitch
    cur_yaw = (ending_yaw - starting_yaw) * time_pct + starting_yaw

    return (cur_x, cur_y, cur_z), (cur_roll, cur_pitch, cur_yaw)


def interpolate_joint_angles(
    t: float,
    starting_time: float,
    duration: float,
    starting_joint_angles: Tuple[float],
    ending_joint_angles: Tuple[float],
) -> Tuple[float]:
    """Function to calculate desired joint angles for a given moment in time given parameters

    Args:
        t (float): _description_
        starting_time (float): _description_
        duration (float): _description_
        starting_joint_angles (Tuple[float]): _description_
        ending_joint_angles (Tuple[float]): _description_

    Returns:
        Tuple[float]: joint angles, 6 dimensions
    """
    ending_time = starting_time + duration
    time_pct = (t - starting_time) / (ending_time - starting_time)

    target_angles = []

    for i in range(len(starting_joint_angles)):
        start_angle = starting_joint_angles[i]
        end_angle = ending_joint_angles[i]
        new_angle = (end_angle - start_angle) * time_pct + start_angle
        target_angles.append(new_angle)
    return target_angles


class TrajectoryPlanner:

    def __init__(self, loop_rate: int = 10):
        """Class to plan a trajectory and cache a calculated trajectory in ros_arm_controller.constants.CACHE_DIR directory

        Args:
            loop_rate (int, optional): How many waypoints per second. Higher results in longer calculations. Defaults to 10.
        """

        self.loop_rate = loop_rate
        self.dt = 1 / self.loop_rate
        self.start_time = 0.0
        self.rpy = [2.75, 0.0, 0.0]

        self.starting_x = 0.4
        self.starting_y = 0.2
        self.x_diff = 0.05
        self.y_diff = 0.025

        self.starting_z = 0.025
        self.paper_height = PAPER_HEIGHT

        self.path_to_urdf = get_urdf_path()

        self.params = PybulletSimParams(
            urdf_file_path=self.path_to_urdf,
            dof_names=DOF_NAMES,
            render=False,
            n_dof=len(DOF_NAMES),
        )

        self.sim = PybulletSim(self.params)
        self.sim.setup()
        ori_targ_quat = pb.getQuaternionFromEuler(self.rpy)
        self.initial_guess = JOINT_REST_ANGLES
        self.first_joint_angle_pose, _ = self.sim.get_ik(
            LINK_NAME,
            [self.starting_x, self.starting_y, self.starting_z],
            targetOri=ori_targ_quat,
            cur_angles=self.initial_guess,
        )

        xyz_goal, link_world_ori_quat, link_lin_vel, link_ang_vel = (
            self.sim.get_link_state(LINK_NAME, self.first_joint_angle_pose)
        )

        ori_goal = pb.getEulerFromQuaternion(link_world_ori_quat)

        # will make letters M
        # first and last segments are special. We interpolate to our starting point on the trajectory in joint angle space to save calculation time of IK
        # last segment we will go back to that starting point from wherever trajectory ends...
        self.traj_key_segments = [
            TrajSeg(
                "start_go_to_start_pos",
                4.0,
                interpolate_joint_angles=True,
                fn_kwargs={
                    "starting_joint_angles": None,
                    "ending_joint_angles": self.first_joint_angle_pose,
                },
                fn=interpolate_joint_angles,
            ),
            TrajSeg(
                "go_to_start_z",
                5.0,
                interpolate_joint_angles=False,
                fn_kwargs={
                    "starting_x": self.starting_x,
                    "ending_x": self.starting_x,
                    "starting_y": self.starting_y,
                    "ending_y": self.starting_y,
                    "starting_z": self.starting_z,
                    "ending_z": self.paper_height,
                    "starting_roll": self.rpy[0],
                    "ending_roll": self.rpy[0],
                    "starting_pitch": self.rpy[1],
                    "ending_pitch": self.rpy[1],
                    "starting_yaw": self.rpy[2],
                    "ending_yaw": self.rpy[2],
                },
                fn=draw_line,
            ),
            TrajSeg(
                "M_left_up",
                5.0,
                interpolate_joint_angles=False,
                fn_kwargs={
                    "starting_x": self.starting_x,
                    "ending_x": self.starting_x + self.x_diff,
                    "starting_y": self.starting_y,
                    "ending_y": self.starting_y - self.y_diff * 1,
                    "starting_z": self.paper_height,
                    "ending_z": self.paper_height,
                    "starting_roll": self.rpy[0],
                    "ending_roll": self.rpy[0],
                    "starting_pitch": self.rpy[1],
                    "ending_pitch": self.rpy[1],
                    "starting_yaw": self.rpy[2],
                    "ending_yaw": self.rpy[2],
                },
                fn=draw_line,
            ),
            TrajSeg(
                "M_middle_down",
                5.0,
                interpolate_joint_angles=False,
                fn_kwargs={
                    "starting_x": self.starting_x + self.x_diff,
                    "ending_x": self.starting_x + self.x_diff / 2,
                    "starting_y": self.starting_y - self.y_diff * 1,
                    "ending_y": self.starting_y - self.y_diff * 2,
                    "starting_z": self.paper_height,
                    "ending_z": self.paper_height,
                    "starting_roll": self.rpy[0],
                    "ending_roll": self.rpy[0],
                    "starting_pitch": self.rpy[1],
                    "ending_pitch": self.rpy[1],
                    "starting_yaw": self.rpy[2],
                    "ending_yaw": self.rpy[2],
                },
                fn=draw_line,
            ),
            TrajSeg(
                "M_middle_up",
                5.0,
                interpolate_joint_angles=False,
                fn_kwargs={
                    "starting_x": self.starting_x + self.x_diff / 2,
                    "ending_x": self.starting_x + self.x_diff,
                    "starting_y": self.starting_y - self.y_diff * 2,
                    "ending_y": self.starting_y - self.y_diff * 3,
                    "starting_z": self.paper_height,
                    "ending_z": self.paper_height,
                    "starting_roll": self.rpy[0],
                    "ending_roll": self.rpy[0],
                    "starting_pitch": self.rpy[1],
                    "ending_pitch": self.rpy[1],
                    "starting_yaw": self.rpy[2],
                    "ending_yaw": self.rpy[2],
                },
                fn=draw_line,
            ),
            TrajSeg(
                "M_right_down",
                5.0,
                interpolate_joint_angles=False,
                fn_kwargs={
                    "starting_x": self.starting_x + self.x_diff,
                    "ending_x": self.starting_x,
                    "starting_y": self.starting_y - self.y_diff * 3,
                    "ending_y": self.starting_y - self.y_diff * 4,
                    "starting_z": self.paper_height,
                    "ending_z": self.paper_height,
                    "starting_roll": self.rpy[0],
                    "ending_roll": self.rpy[0],
                    "starting_pitch": self.rpy[1],
                    "ending_pitch": self.rpy[1],
                    "starting_yaw": self.rpy[2],
                    "ending_yaw": self.rpy[2],
                },
                fn=draw_line,
            ),
            TrajSeg(
                "pen_up",
                1.0,
                interpolate_joint_angles=False,
                fn_kwargs={
                    "starting_x": self.starting_x,
                    "ending_x": self.starting_x,
                    "starting_y": self.starting_y - self.y_diff * 4,
                    "ending_y": self.starting_y - self.y_diff * 4,
                    "starting_z": self.paper_height,
                    "ending_z": self.starting_z,
                    "starting_roll": self.rpy[0],
                    "ending_roll": self.rpy[0],
                    "starting_pitch": self.rpy[1],
                    "ending_pitch": self.rpy[1],
                    "starting_yaw": self.rpy[2],
                    "ending_yaw": self.rpy[2],
                },
                fn=draw_line,
            ),
            TrajSeg(
                "end_go_to_start_pos",
                4.0,
                interpolate_joint_angles=True,
                fn_kwargs={
                    "starting_joint_angles": None,
                    "ending_joint_angles": JOINT_REST_ANGLES,
                },
                fn=interpolate_joint_angles,
            ),
        ]

        self.durations = [t.duration for t in self.traj_key_segments]
        self.n_traj = len(self.durations)

        self.traj_key_times = []
        for i in range(self.n_traj):

            duration_up_to = 0.0
            for j in range(self.n_traj):
                if i == j:
                    break
                duration_up_to += self.durations[j]
            self.traj_key_times.append(self.start_time + duration_up_to)

        self.cache_dir = CACHE_DIR
        self.cache_file = os.path.join(self.cache_dir, "traj.pickle")

    def get_transform_last_frame(self, joint_angles: Tuple[float]):
        xyz_world, link_world_ori_quat, link_lin_vel, link_ang_vel = (
            self.sim.get_link_state(LINK_NAME, joint_angles)
        )

        ori_euler = pb.getEulerFromQuaternion(link_world_ori_quat)

        return xyz_world, ori_euler

    def get_target(
        self, t: float, prior_joint_angles: Optional[Tuple[float]] = None
    ) -> Tuple[
        str, Tuple[float], Tuple[float], Tuple[float], Tuple[float], Tuple[float], bool
    ]:
        """Helper function to get a target from a given time instance by finding the relevant trajectory segment.

        Args:
            t (float): time instance
            prior_joint_angles (Optional[Tuple[float]], optional): prior joint angles. Defaults to None.

        Returns:
            Tuple[ str, Tuple[float], Tuple[float], Tuple[float], Tuple[float], Tuple[float], bool]: name of the trajectory segment, xyz goal in 3 dimensions meters, orientation goal 3 dimensions euler angles radians, target angles of the arm 6 dimensions, calculated xyz post IK 3 dimensions, calculated orientation post IK 3 dimensions, whether it converged
        """

        traj_segment_idx = bisect_left(self.traj_key_times, t)
        # if we ask to find idx of 0.0 and it's in our array it'll give us that idx, else it'll give us idx after to insert into
        if (
            traj_segment_idx == len(self.traj_key_times)
            or self.traj_key_times[traj_segment_idx] > t
        ):
            traj_segment_idx -= 1
        traj_segment = self.traj_key_segments[traj_segment_idx]
        start_time = self.traj_key_times[traj_segment_idx]
        duration = self.durations[traj_segment_idx]
        name = traj_segment.name

        converged = True

        if traj_segment.interpolate_joint_angles:
            # for first and last traj segment we will fill in the prior angles as the starting point
            if traj_segment.fn_kwargs["starting_joint_angles"] is None:
                if prior_joint_angles is None:
                    prior_joint_angles = JOINT_REST_ANGLES
                traj_segment.fn_kwargs["starting_joint_angles"] = prior_joint_angles
            angles_targ = interpolate_joint_angles(
                t, starting_time=start_time, duration=duration, **traj_segment.fn_kwargs
            )

            xyz_goal, ori_goal = self.get_transform_last_frame(angles_targ)
            xyz_calc = copy.copy(xyz_goal)
            ori_calc = copy.copy(ori_goal)
        else:
            xyz_goal, ori_goal = traj_segment.fn(
                t, starting_time=start_time, duration=duration, **traj_segment.fn_kwargs
            )
            ori_targ_quat = pb.getQuaternionFromEuler(ori_goal)
            angles_targ, success = self.sim.get_ik(
                LINK_NAME,
                xyz_goal,
                targetOri=ori_targ_quat,
                cur_angles=prior_joint_angles,
            )
            converged = converged and success
            xyz_calc, ori_calc = self.get_transform_last_frame(angles_targ)
        return name, xyz_goal, ori_goal, angles_targ, xyz_calc, ori_calc, converged

    def make_joint_traj(self, cur_angles: Tuple[float]):
        """Function to take in current angles and calculate the entire trajectory by looping over time steps and getting trajectory segments.
        Caches trajectory in ros_arm_controller.constants.CACHE_DIR directory

        Args:
            cur_angles (Tuple[float]): current angles of the robot
        Returns:
            bool: Whether all solves converged succesfully
        """

        if os.path.exists(self.cache_file):
            with open(self.cache_file, "rb") as file_handle:
                data = pickle.load(file_handle)
            self.traj_times = data["times"]
            self.traj_angles = data["angles_traj"]
            self.xyz_goals = data["xyz_goals"]
            self.ori_goals = data["ori_goals"]
            self.xyz_calc = data["xyz_calc"]
            self.ori_calc = data["ori_calc"]
            self.names = data["names"]
            return

        cur_time = self.start_time
        # we adjust the starting segment such that it starts from current xzy

        cur_xyz, cur_rpy = self.get_transform_last_frame(cur_angles)
        end_time = self.traj_key_times[-1] + self.durations[-1]

        angles_traj = []
        xyz_goals = []
        ori_goals = []
        xyz_calculated = []
        ori_calculated = []
        times = []
        names = []

        num_its = int((end_time - cur_time) / self.dt)

        last_seg = False

        all_converged = True

        for i in tqdm(range(num_its)):
            name, xyz_targ, ori_targ, angles_targ, xyz_calc, ori_calc, success = (
                self.get_target(cur_time, prior_joint_angles=cur_angles)
            )
            angles_traj.append(copy.deepcopy(angles_targ))
            times.append(cur_time)
            xyz_goals.append(xyz_targ)
            ori_goals.append(ori_targ)
            xyz_calculated.append(xyz_calc)
            ori_calculated.append(ori_calc)
            names.append(name)
            cur_time += self.dt
            cur_angles = angles_targ
            all_converged = all_converged and success

        self.traj_times = times
        self.traj_angles = angles_traj
        self.xyz_goals = xyz_goals
        self.ori_goals = ori_goals
        self.xyz_calc = xyz_calculated
        self.ori_calc = ori_calculated
        self.names = names

        data = {
            "times": times,
            "angles_traj": angles_traj,
            "xyz_goals": xyz_goals,
            "ori_goals": ori_goals,
            "xyz_calc": xyz_calculated,
            "ori_calc": ori_calculated,
            "names": names,
        }
        with open(self.cache_file, "wb") as file_handle:
            pickle.dump(data, file_handle, pickle.HIGHEST_PROTOCOL)

        (
            times,
            joint_angles,
            xyzs_planned,
            oris_planned,
            xyzs_solved,
            oris_solved,
        ) = self.get_xyz_full_traj()

        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")

        arrs_to_plot = [xyzs_planned, xyzs_solved]
        labels = ["planned_traj", "solved_traj"]

        for i in range(len(arrs_to_plot)):
            arr = arrs_to_plot[i]
            lab = labels[i]
            arr = np.array(arr)
            x = arr[:, 0] if len(arr) > 1 else []
            y = arr[:, 1] if len(arr) > 1 else []
            z = arr[:, 2] if len(arr) > 1 else []
            ax.plot(x, y, z, label=lab)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_xlim(*X_PLOT_LIM)
        ax.set_ylim(*Y_PLOT_LIM)
        ax.set_zlim(*Z_PLOT_LIM)
        ax.legend()

        # rotation around z, degrees
        ax.azim = 180
        # distance from center point
        ax.dist = 0.15
        # angle between eye and xy plane
        ax.elev = 45

        print(f"saving plot to: {self.cache_dir}")
        fig.savefig(os.path.join(self.cache_dir, "traj_plan_vs_solved.png"))

        fig, axes = plt.subplots(3, 1, sharex=True, sharey=True)

        times_arr = np.array(times)
        ori_arr_planned = np.array(oris_planned)
        ori_arr_solved = np.array(oris_solved)

        labels = ["rx", "ry", "rz"]

        for i in range(3):
            lab = labels[i]
            axes[i].plot(times_arr, ori_arr_planned[:, i], label=lab + "_plan")
            axes[i].plot(times_arr, ori_arr_solved[:, i], label=lab + "_solved")
            axes[i].set_ylabel("euler angle (radians)")
            axes[i].legend(loc="lower right")

        axes[-1].set_xlabel("time")
        fig.savefig(os.path.join(self.cache_dir, "traj_ori_plan_vs_solved.png"))

        joint_angles = np.array(joint_angles)
        fig, axes = plt.subplots(joint_angles.shape[1], 1, sharex=True, sharey=True)
        for i in range(joint_angles.shape[1]):
            axes[i].plot(times_arr, joint_angles[:, i], label=f"joint_{i}")
            axes[i].set_ylabel("angle, rad.")
        axes[-1].set_xlabel("time")
        fig.legend()
        fig.savefig(os.path.join(self.cache_dir, "traj_joint_angles.png"))

        return all_converged

    def get_xyz_full_traj(self):
        """Helper function to get key details of the trajectory for plotting

        Returns:
            _type_: _description_
        """
        return (
            self.traj_times,
            self.traj_angles,
            self.xyz_goals,
            self.ori_goals,
            self.xyz_calc,
            self.ori_calc,
        )

    def get_joint_angles(self, t: float):
        """Helper function to get joint angles and other details from a calculated trajectory. Must be called post calculation.

        Args:
            t (float): time step

        Returns:
            _type_: _description_
        """
        traj_idx = bisect_left(self.traj_times, t)
        if traj_idx == len(self.traj_times) or self.traj_times[traj_idx] > t:
            traj_idx -= 1
        joint_angles = self.traj_angles[traj_idx]
        xyz = self.xyz_goals[traj_idx]
        ori = self.ori_goals[traj_idx]
        name = self.names[traj_idx]
        return name, joint_angles, xyz, ori, traj_idx == len(self.traj_angles) - 1
