from abc import ABC, abstractmethod
from enum import Enum
import math
from typing import Tuple, Optional

import random


class DOFControlMode(Enum):
    """Enumeration for how to control joints"""

    POSITION_CONTROL = 0


class SimParams:
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
        dt=0.02,
    ):
        """Simulation params, common across simulators

        Args:
            n_dof (int, optional): degrees of freedom of the system. Defaults to 6.
            dof_names (Tuple[str], optional): names of the joints. Defaults to ( "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", ).
            dof_control_mode (DOFControlMode, optional): control mode of the system. Defaults to DOFControlMode.POSITION_CONTROL.
            render (bool, optional): whether to render and open a GUI. Defaults to True.
            dt (float, optional): timestep for the simulation. Defaults to 0.02.
        """

        self.n_dof = n_dof
        self.dof_names = dof_names
        self.dof_control_mode = dof_control_mode
        self.render = render
        self.dt = dt


class Simulator(ABC):
    def __init__(self, sim_params: SimParams):
        """initialize the simulator

        Args:
            sim_params (SimParams): parameters of the system

        Raises:
            NotImplementedError: if bad arguments
        """
        assert sim_params.n_dof == len(sim_params.dof_names)
        self.n_dof = sim_params.n_dof
        self.dof_names = sim_params.dof_names
        self.render = sim_params.render

        if sim_params.dof_control_mode != DOFControlMode.POSITION_CONTROL:
            raise NotImplementedError

        self.dof_control_mode = sim_params.dof_control_mode

    @abstractmethod
    def get_joint_pos(self) -> Tuple[float]:
        """Get the position of the joint that is cached from the last timestep

        Returns:
            Tuple[float]: position
        """

        pass

    @abstractmethod
    def step(self, control: Tuple[float]) -> None:
        """Step the simulation taking the control as an input and update the joint state cache.

        Args:
            control (Tuple[float]): control input
        """

    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def close(self):
        """close the simulation"""
        pass

    @abstractmethod
    def reset(self):
        """Reset the simulation"""
        pass

    @abstractmethod
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


if __name__ == "__main__":
    pass
