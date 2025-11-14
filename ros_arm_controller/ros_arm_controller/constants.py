import math

# this directory is where plots will be saved and the trajectory as a binary file
CACHE_DIR = "/home/developer/ros_ws/src/robo_drawing"
DEFAULT_EPS_XYZ = 0.002
DEFAULT_EPS_ORI = 0.01
JOINT_ANGLE_DIFF_TOL = 2 * math.pi / 180  # 5 degrees?
PAPER_HEIGHT = 0.007  # in sim
# PAPER_HEIGHT = -0.001  # in reality
# intrinsic
ROTATION_SCHEMA = "xyz"

X_PLOT_LIM = (0.0, 0.25)
Y_PLOT_LIM = (-0.1, 0.3)
Z_PLOT_LIM = (-0.005, 1.0)
