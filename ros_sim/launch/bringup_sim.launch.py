import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "robot_name",
            default_value="mycobot_280",
            description="Robot name to load, either mycobot_280 or franka_panda.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "base_link",
            default_value="base",
            description="The base link of the robot. We will publish static transform from the world to this, as identity.",
        )
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        parameters=[{"use_sim_time": True}],
        output="log",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            LaunchConfiguration("base_link"),
        ],
    )
    ld.add_action(static_tf)

    sim_node = Node(
        package="ros_sim",
        executable="ros_sim",
        output="screen",
        parameters=[{"robot_name": LaunchConfiguration("robot_name")}],
    )
    ld.add_action(sim_node)

    return ld
