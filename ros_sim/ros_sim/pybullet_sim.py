import copy
import math
from typing import Tuple, Optional

import numpy as np
import pybullet as pb
import pybullet_data
from pybullet_utils import bullet_client

from ros_sim.sim import SimParams, Simulator, DOFControlMode
from ros_sim.constants import (
    MAX_FORCES,
    MAX_FORCE,
    JOINT_DAMPING_COEF,
    POS_GAIN,
    VEL_GAIN,
    LOWER_LIMITS,
    UPPER_LIMITS,
    JOINT_RANGE,
    JOINT_REST_ANGLES,
)


class PybulletSimParams(SimParams):
    def __init__(
        self,
        n_dof: int = 6,
        dof_names: Tuple[str] = (
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ),
        dof_control_mode: DOFControlMode = DOFControlMode.POSITION_CONTROL,
        render: bool = True,
        urdf_file_path: str = "",
        sim_dt: float = 0.005,
    ):
        """A container class to hold parameters used in the pybullet simulation, in addition to the ones in the baseclass.

        Args:
            n_dof (int, optional): degrees of freedom of the system to be simulated. Defaults to 6.
            dof_names (Tuple[str], optional): names of the joints of the system to be simulated. Defaults to ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6").
            dof_control_mode (DOFControlMode, optional): how to control the joints. Defaults to DOFControlMode.POSITION_CONTROL.
            render (bool, optional): whether to render by opening a GUI. Defaults to True.
            urdf_file_path (str, optional): what path to the robot description file. This needs to be processed to remove package names from mesh files. Defaults to "".
            sim_dt (float, optional): timestep for the simulation. Defaults to 0.005.

        Raises:
            ValueError: if any bad params
        """

        super().__init__(n_dof, dof_names, dof_control_mode, render)

        self.urdf_file_path = urdf_file_path
        self.sim_dt = sim_dt
        if self.dt / self.sim_dt - int(self.dt / self.sim_dt) != 0.0:
            raise ValueError(
                f"dt {self.dt} must be evenly divisible by sim_dt {self.sim_dt}"
            )
        self.decimation = int(self.dt / self.sim_dt)


class PybulletSim(Simulator):

    def __init__(self, params: PybulletSimParams):
        """_summary_

        Args:
            params (PybulletSimParams): parameters for the simulation
        """
        super().__init__(params)

        self.urdf_file_path = params.urdf_file_path
        self.decimation = params.decimation
        self.sim_dt = params.sim_dt

        self.dof_names = params.dof_names

        self.joint_indices = []
        self.link_name_to_id = {}

        self.__joint_pos = [0.0 for i in range(self.n_dof)]
        self.__joint_vel = [0.0 for i in range(self.n_dof)]
        self.__joint_torque = [0.0 for i in range(self.n_dof)]

    def __buildLinkNameToId(self):
        """
        Helper function to populate a datastructure to go from joint name to pybullet joint index.
        """

        num_joints = self.pybullet_client.getNumJoints(self.bot_pybullet)
        all_joint_names = []
        all_joint_indices = []
        for i in range(num_joints):
            joint_info = self.pybullet_client.getJointInfo(self.bot_pybullet, i)
            joint_index = joint_info[0]
            joint_name = joint_info[1].decode("UTF-8")
            link_name = joint_info[12].decode("UTF-8")
            parent_link_index = joint_info[16]
            self.link_name_to_id[link_name] = joint_index
            self.joint_name_to_id[joint_name] = joint_index

            all_joint_names.append(joint_name)
            all_joint_indices.append(i)
        for i in range(num_joints):
            joint_name = all_joint_names[i]
            joint_idx = all_joint_indices[i]
            if joint_name in self.dof_names:
                self.joint_indices.append(joint_idx)

    def setup(self):
        """Initialize the simulation"""

        if self.render:
            mode = pb.GUI
        else:
            mode = pb.DIRECT

        self.pybullet_client = bullet_client.BulletClient(connection_mode=mode)
        self.pybullet_client.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
        self.pybullet_client.setPhysicsEngineParameter(numSolverIterations=30)
        # note pybullet does not recommend changing the timestep from their default of 240HZ.
        self.pybullet_client.setTimeStep(self.sim_dt)
        self.pybullet_client.setGravity(0, 0, -9.81)
        self.pybullet_client.setPhysicsEngineParameter(enableConeFriction=0)
        self.pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.ground_body = self.pybullet_client.loadURDF("plane.urdf")

        flags = (
            self.pybullet_client.URDF_USE_INERTIA_FROM_FILE
            | self.pybullet_client.URDF_USE_SELF_COLLISION
        )
        self.bot_pybullet = self.pybullet_client.loadURDF(
            self.urdf_file_path, [0, 0, 0], [0, 0, 0, 1], useFixedBase=True, flags=flags
        )

        self.link_name_to_id = {}
        self.joint_name_to_id = {}
        self.__buildLinkNameToId()

        # pybullet:
        # Important Note: by default, each revolute joint and prismatic joint is motorized using a velocity
        # motor. You can disable those default motor by using a maximum force of 0. This will let you
        # perform torque control. You can also use a small non-zero force to mimic joint friction.
        # frictionForce = 0.2
        # mode = pb.VELOCITY_CONTROL
        # for joint_idx in self.joint_indices:
        #     self.pybullet_client.setJointMotorControl2(bodyUniqueId=self.bot_pybullet, jointIndex=joint_idx,
        #             controlMode=mode,targetVelocity=0, force=frictionForce)

        for joint_idx in self.joint_indices:
            # disable default constraint-based motors
            self.pybullet_client.setJointMotorControl2(
                bodyUniqueId=self.bot_pybullet,
                jointIndex=joint_idx,
                controlMode=pb.POSITION_CONTROL,
                targetPosition=0,
                force=MAX_FORCE,
            )

        self.reset()

        self.__joint_pos, self.__joint_vel, self.__joint_torque = (
            self.get_joint_pos_vel()
        )

        for joint_idx in self.joint_indices:
            joint_info = self.pybullet_client.getJointInfo(self.bot_pybullet, joint_idx)

        camera_distance = 2.0
        camera_yaw = 0.0  # deg
        camera_pitch = -20  # deg
        camera_target_position = [0.0, 0.0, 0.0]
        self.pybullet_client.resetDebugVisualizerCamera(
            camera_distance, camera_yaw, camera_pitch, camera_target_position
        )

    def close(self):
        """close the simulation"""
        pass

    def reset(self):
        """Reset the simulation"""
        self.pybullet_client.resetBasePositionAndOrientation(
            self.bot_pybullet, [0, 0, 0], [0, 0, 0, 1]
        )

        for i in self.joint_indices:
            self.pybullet_client.resetJointState(self.bot_pybullet, i, 0.0)

    def get_joint_pos_vel(self) -> Tuple[Tuple[float], Tuple[float], Tuple[float]]:
        """Get the state of a particular joint including position, velocity, and torque

        Returns:
            Tuple[Tuple[float], Tuple[float], Tuple[float]]: position, velocity, and torque
        """
        joint_states = self.pybullet_client.getJointStates(
            bodyUniqueId=self.bot_pybullet, jointIndices=self.joint_indices
        )
        joint_pos = []
        joint_vel = []
        joint_torque = []
        for i in range(self.n_dof):
            joint_state = joint_states[i]
            joint_pos.append(joint_state[0])
            joint_vel.append(joint_state[1])
            joint_torque.append(joint_state[3])

        return joint_pos, joint_vel, joint_torque

    def get_joint_pos(self) -> Tuple[float]:
        """Get the position of the joint that is cached from the last timestep

        Returns:
            Tuple[float]: position
        """
        return copy.copy(self.__joint_pos)

    def get_joint_tau(self) -> Tuple[float]:
        """Get the torque of the joint that is cached from the last timestep

        Returns:
            Tuple[float]: torque
        """
        return copy.copy(self.__joint_torque)

    def step(self, control: Tuple[float]) -> None:
        """Step the simulation taking the control as an input and update the joint state cache.

        Args:
            control (Tuple[float]): control input
        """
        if POS_GAIN is not None and VEL_GAIN is not None:
            self.pybullet_client.setJointMotorControlArray(
                bodyIndex=self.bot_pybullet,
                jointIndices=self.joint_indices,
                controlMode=self.pybullet_client.POSITION_CONTROL,
                targetPositions=control,
                forces=MAX_FORCES,
                targetVelocities=[0 for i in range(self.n_dof)],
                positionGains=POS_GAIN,
                velocityGains=VEL_GAIN,
            )
        else:
            self.pybullet_client.setJointMotorControlArray(
                bodyIndex=self.bot_pybullet,
                jointIndices=self.joint_indices,
                controlMode=self.pybullet_client.POSITION_CONTROL,
                targetPositions=control,
                forces=MAX_FORCES,
                targetVelocities=[0 for i in range(self.n_dof)],
            )

        for i in range(self.decimation):
            self.pybullet_client.stepSimulation()

        self.__joint_pos, self.__joint_vel, self.__joint_torque = (
            self.get_joint_pos_vel()
        )

    def get_link_state(
        self, link_name: str, joint_angles: Optional[Tuple[float]] = None
    ) -> Tuple[Tuple[float], Tuple[float], Tuple[float], Tuple[float]]:
        """Get the state of a link given a name from the URDF file

        Args:
            link_name (str): link name from URDF
            joint_angles (Optional[Tuple[float]]): joint angles where if given it will perform FK.

        Returns:
            Tuple[Tuple[float], Tuple[float], Tuple[float], Tuple[float]]: world position, world orientation (quaternion), link velocity, link angular velocity
        """
        link_idx = self.link_name_to_id[link_name]

        prior_joint_angles = copy.copy(self.__joint_pos)
        if joint_angles is not None:
            for i in range(len(joint_angles)):
                joint_idx = self.joint_indices[i]
                self.pybullet_client.resetJointState(
                    self.bot_pybullet, joint_idx, joint_angles[i]
                )

        link_state = self.pybullet_client.getLinkState(
            bodyUniqueId=self.bot_pybullet,
            linkIndex=link_idx,
            computeLinkVelocity=True,
            computeForwardKinematics=True,
        )
        link_world_pos, link_world_ori, link_lin_vel, link_ang_vel = (
            link_state[4],
            link_state[5],
            link_state[6],
            link_state[7],
        )
        if joint_angles is not None:
            for i in range(len(prior_joint_angles)):
                joint_idx = self.joint_indices[i]
                self.pybullet_client.resetJointState(
                    self.bot_pybullet, joint_idx, prior_joint_angles[i]
                )
        return link_world_pos, link_world_ori, link_lin_vel, link_ang_vel

    def get_link_jacobian(
        self, link_name: str, joint_positions: Tuple[float]
    ) -> Tuple[Tuple[float], Tuple[float]]:
        """Get the jacobian for a given link name at a given joint position.

        Args:
            link_name (str): link name from URDF
            joint_positions (Tuple[float]): position of the joint

        Returns:
            Tuple[Tuple[float], Tuple[float]]: linear jacobian, angular jacobian
        """

        link_idx = self.link_name_to_id[link_name]
        linear_jacobian, angular_jacobian = self.pybullet_client.calculateJacobian(
            bodyUniqueId=self.bot_pybullet,
            linkIndex=link_idx,
            localPosition=[0, 0, 0],
            objPositions=joint_positions,
            objVelocities=[0 for i in range(self.n_dof)],
            objAccelerations=[0 for i in range(self.n_dof)],
        )
        return linear_jacobian, angular_jacobian

    def get_ik(
        self,
        link_name: str,
        target_xyz: Tuple[float],
        cur_angles: Tuple[float],
        targetOri: Optional[Tuple[float]] = None,
        threshold: float = 0.0001,
        max_it: int = 5,
    ) -> Tuple[Tuple[float], bool]:
        """get inverse kinematics by using Pybullet's solver.

        Args:
            link_name (str): link name from URDF
            target_xyz (Tuple[float]): target xyz point of link
            cur_angles (Tuple[float]): current joint angules of system to linearize about
            targetOri (Optional[Tuple[float]], optional): target orientation in quaternion form, may be None to just do position. Defaults to None.
            threshold (float, optional): threshold for XYZ distance. Note that if a position outside the workspace is passed as a goal, may not be satisfied. Defaults to 0.0001.
            max_it (int, optional): max number of iterations to call pybullet's solver. Defaults to 5.

        Returns:
            Tuple[Tuple[float], bool]: solved angles, whether solution is within position tolerance
        """
        link_idx = self.link_name_to_id[link_name]

        prior_joint_angles = copy.copy(self.__joint_pos)
        for i in range(len(cur_angles)):
            joint_idx = self.joint_indices[i]
            self.pybullet_client.resetJointState(
                self.bot_pybullet, joint_idx, cur_angles[i]
            )
        close_enough = False
        num_it = 0
        while not close_enough and num_it < max_it:
            joint_angles = self.pybullet_client.calculateInverseKinematics(
                self.bot_pybullet,
                link_idx,
                targetPosition=target_xyz,
                targetOrientation=targetOri,
                jointDamping=JOINT_DAMPING_COEF,
                lowerLimits=LOWER_LIMITS,
                upperLimits=UPPER_LIMITS,
                jointRanges=JOINT_RANGE,
                restPoses=JOINT_REST_ANGLES,
            )

            for i in range(len(joint_angles)):
                joint_idx = self.joint_indices[i]
                self.pybullet_client.resetJointState(
                    self.bot_pybullet, joint_idx, joint_angles[i]
                )
            link_state = self.pybullet_client.getLinkState(
                bodyUniqueId=self.bot_pybullet,
                linkIndex=link_idx,
                computeLinkVelocity=True,
                computeForwardKinematics=True,
            )
            new_pos = link_state[4]
            diff = [
                target_xyz[0] - new_pos[0],
                target_xyz[1] - new_pos[1],
                target_xyz[2] - new_pos[2],
            ]
            dist2 = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]
            close_enough = dist2 < threshold
            num_it = num_it + 1

        for i in range(len(prior_joint_angles)):
            joint_idx = self.joint_indices[i]
            self.pybullet_client.resetJointState(
                self.bot_pybullet, joint_idx, prior_joint_angles[i]
            )

        return joint_angles, close_enough
