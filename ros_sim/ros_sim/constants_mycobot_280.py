import numpy as np


LINK_NAME = "wand_tip"
MAX_FORCE = 20.0

DOF_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

JOINT_LIMITS = [
    [-165, 165],
    [-165, 165],
    [-165, 165],
    [-165, 165],
    [-165, 165],
    [-179, 179],
]
JOINT_LIMITS = np.array(JOINT_LIMITS)
JOINT_LIMITS = np.radians(JOINT_LIMITS)

LOWER_LIMITS = np.array(JOINT_LIMITS)[:, 0].tolist()
UPPER_LIMITS = np.array(JOINT_LIMITS)[:, 1].tolist()
JOINT_RANGE = (np.array(JOINT_LIMITS)[:, 1] - np.array(JOINT_LIMITS)[:, 0]).tolist()
JOINT_REST_ANGLES = [0.00, -0.2 * np.pi, -0.2 * np.pi, 0.0, 0.0, 0.0]

URDF_PACKAGE = "ros_sim"
URDF_PATH_WITHIN_PACAKGE = [
    "resource",
    "robots",
    "mycobot_280_pi",
    "mycobot_280_pi_mod.urdf",
]
URDF_PATH_TO_MESHES = ["meshes"]

JOINT_DAMPING_COEF = [1.1] * JOINT_LIMITS.shape[0]

MAX_FORCES = [MAX_FORCE for i in range(JOINT_LIMITS.shape[0])]

POS_GAIN = [0.4 for i in range(len(JOINT_LIMITS))]
VEL_GAIN = [1.0 for i in range(len(JOINT_LIMITS))]
