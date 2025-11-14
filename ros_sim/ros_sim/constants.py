import numpy as np

MAX_FORCE = 50.0

LINK_NAME = "pen_tip"
DOF_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]

JOINT_LIMITS = [
    [-2.7437, 2.7437],
    [-1.7837, 1.7837],
    [-2.9007, 2.9007],
    [-3.0421, -0.1518],
    [-2.8065, 2.8065],
    [0.5445, 4.5169],
    [-3.0159, 3.0159],
]
LOWER_LIMITS = np.array(JOINT_LIMITS)[:, 0].tolist()
UPPER_LIMITS = np.array(JOINT_LIMITS)[:, 1].tolist()
JOINT_RANGE = (np.array(JOINT_LIMITS)[:, 1] - np.array(JOINT_LIMITS)[:, 0]).tolist()
JOINT_REST_ANGLES = [
    0.00,
    -0.25 * np.pi,
    0.00,
    -0.75 * np.pi,
    0.00,
    0.50 * np.pi,
    0.25 * np.pi,
]


JOINT_DAMPING_COEF = [1.1] * len(DOF_NAMES)

MAX_FORCES = [MAX_FORCE for i in range(len(DOF_NAMES))]

POS_GAIN = None
VEL_GAIN = None
