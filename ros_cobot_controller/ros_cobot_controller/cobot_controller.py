import rclpy
from rclpy.node import Node

import time


from sensor_msgs.msg import JointState

import numpy as np

import pymycobot
from pymycobot.mycobot import MyCobot
from pymycobot.error import MyCobotDataException


COBOT_JOINT_GOAL_TOPIC = "/target_joint_states"
COBOT_JOIN_REAL_TOPIC = "/joint_states"

NUM_ANGLES = 6
# the joints have the below min/maxes (from https://www.elephantrobotics.com/en/mycobot-280-pi-2023-specifications/)
# J1 -165 ~ +165
# J2 -165 ~ +165
# J3 -165 ~ +165
# J4 -165 ~ +165
# J5 -165 ~ +165
# J6 -179 ~ +179
JOINT_LIMITS = np.array(
    [[-165, 165], [-165, 165], [-165, 165], [-165, 165], [-165, 165], [-179, 179]]
)


DOF_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]


def check_angles(target_angles):
    are_angles_ok = np.where(target_angles < JOINT_LIMITS[:, 0], 1, 0) + np.where(
        target_angles > JOINT_LIMITS[:, 1], 1, 0
    )
    return np.sum(are_angles_ok) == 0


class CurAngles:
    def __init__(self, angles, speed):
        self.angles = angles
        self.speed = speed

    def __eq__(self, other):
        if isinstance(other, CurAngles):
            return self.angles == other.angles and self.speed == other.speed
        return False

    def __ne__(self, other):
        # needed in python 2
        return not self.__eq__(other)

    def __str__(self):
        return f"angles: {self.angles} speed: {self.speed}"


class MycobotController(Node):

    def __init__(self):
        super().__init__("mycobot_controller")

        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baud", 1000000)
        self.declare_parameter("pub_angle_timer", 0.01)
        self.declare_parameter("mycobot_speed", 100)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value

        self.get_logger().info("start ...")
        self.get_logger().info("Params: %s,%s" % (port, baud))

        pymycobot_version = pymycobot.__version__
        self.get_logger().info("pymycobot version: %s" % (pymycobot_version))

        self.mc = MyCobot(port, baud)

        is_connected = self.mc.is_controller_connected()
        if not is_connected:
            raise RuntimeError(
                "Not connected to the mycobot. Check if the robot is on and the serial port is correct?"
            )

        self.real_angle_pub = self.create_publisher(
            JointState, COBOT_JOIN_REAL_TOPIC, 5
        )

        self.cmd_angle_sub = self.create_subscription(
            JointState, COBOT_JOINT_GOAL_TOPIC, self.cmd_angle_callback, 5
        )
        self.cmd_angle_sub  # prevent unused variable warning

        self.timer_real_angles = self.create_timer(
            self.get_parameter("pub_angle_timer").value,
            self.get_and_publish_real_angles,
        )

        zero_angles = CurAngles([0 for i in range(NUM_ANGLES)], 20)
        self.cur_angles = zero_angles
        self.prev_angles = zero_angles

        self.set_cur_cmd_angles()

        time.sleep(2.0)

    def cmd_angle_callback(self, msg):
        cur_cmd_angles = msg.position
        cur_cmd_speed = (
            self.get_parameter("mycobot_speed").get_parameter_value().integer_value
        )
        cmd_angles_arr = np.array(cur_cmd_angles)
        cmd_angles_arr = cmd_angles_arr * 180 / np.pi
        angles_ok = check_angles(cmd_angles_arr)
        if not angles_ok:
            self.get_logger().error(
                f"command angles invalid. Commanded:\n{np.array_str(cmd_angles_arr)}\nLimits:\n{np.array_str(JOINT_LIMITS)}"
            )
            return

        self.cur_angles = CurAngles(cmd_angles_arr.tolist(), cur_cmd_speed)
        # print(f"cur angles: {self.cur_angles.angles} prev angles {self.prev_angles.angles}")
        if self.cur_angles != self.prev_angles:
            self.set_cur_cmd_angles()

    def get_and_publish_real_angles(self):
        msg = JointState()
        self.get_logger().debug("reading angles")
        angles = self.mc.get_angles()
        if angles is None or angles[0] == angles[1] == angles[2] == 0.0:
            self.get_logger().error("angles came back None or 0.0, {}".format(angles))
            return
        self.get_logger().debug("read angles")
        for i in range(len(angles)):
            angles_rad = angles[i] * np.pi / 180
            name = DOF_NAMES[i]
            msg.position.append(angles_rad)
            msg.name.append(name)
        self.real_angle_pub.publish(msg)
        self.get_logger().debug("published angles")

    def set_cur_cmd_angles(self):
        self.get_logger().debug("sending cmd angles")
        try:
            self.mc.send_angles(self.cur_angles.angles, self.cur_angles.speed)
            self.prev_angles = self.cur_angles
            self.get_logger().debug("sent cmd angles")
        except MyCobotDataException as err:
            self.get_logger().error(
                "invalid joint command. Command was {}, error was {}".format(
                    self.cur_angles.angles, err
                )
            )
            self.cur_angles = self.prev_angles


def main(args=None):
    rclpy.init(args=args)

    mycobot_controller = MycobotController()

    rclpy.spin(mycobot_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mycobot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
