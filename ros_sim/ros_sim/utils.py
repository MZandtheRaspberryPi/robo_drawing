import os
from typing import Tuple

from ament_index_python.packages import get_package_share_directory


def import_constants(robot_name: str):
    # this should all be in a config file, not imported...
    if robot_name == "mycobot_280":
        from ros_sim.constants_mycobot_280 import (
            MAX_FORCE,
            LINK_NAME,
            DOF_NAMES,
            JOINT_LIMITS,
            LOWER_LIMITS,
            UPPER_LIMITS,
            JOINT_RANGE,
            JOINT_REST_ANGLES,
            URDF_PACKAGE,
            URDF_PATH_WITHIN_PACAKGE,
            URDF_PATH_TO_MESHES,
            JOINT_DAMPING_COEF,
            MAX_FORCES,
            POS_GAIN,
            VEL_GAIN,
        )
    elif robot_name == "franka_panda":
        from ros_sim.constants_franka_panda import (
            MAX_FORCE,
            LINK_NAME,
            DOF_NAMES,
            JOINT_LIMITS,
            LOWER_LIMITS,
            UPPER_LIMITS,
            JOINT_RANGE,
            JOINT_REST_ANGLES,
            URDF_PACKAGE,
            URDF_PATH_WITHIN_PACAKGE,
            URDF_PATH_TO_MESHES,
            JOINT_DAMPING_COEF,
            MAX_FORCES,
            POS_GAIN,
            VEL_GAIN,
        )
    else:
        raise NotImplementedError

    return (
        MAX_FORCE,
        LINK_NAME,
        DOF_NAMES,
        JOINT_LIMITS,
        LOWER_LIMITS,
        UPPER_LIMITS,
        JOINT_RANGE,
        JOINT_REST_ANGLES,
        URDF_PACKAGE,
        URDF_PATH_WITHIN_PACAKGE,
        URDF_PATH_TO_MESHES,
        JOINT_DAMPING_COEF,
        MAX_FORCES,
        POS_GAIN,
        VEL_GAIN,
    )


def replace_old_path_urdf(urdf_file_path: str, old_path: str, new_path: str):
    file_str = None
    with open(urdf_file_path, "r") as file_handle:
        file_str = file_handle.read()

    with open(urdf_file_path, "w") as file_handle:

        new_file_str = file_str.replace(old_path, new_path)

        file_handle.write(new_file_str)


def get_urdf_path(
    urdf_package: str, path_to_urdf: Tuple[str], path_to_mesh: Tuple[str]
):
    package_share_directory = get_package_share_directory(urdf_package)
    path_to_urdf = os.path.join(package_share_directory, *path_to_urdf)
    path_to_mesh = os.path.join(package_share_directory, *path_to_mesh)

    old_file = ""
    with open(path_to_urdf, "r") as file_handle:
        old_file = file_handle.read()

    path_to_urdf = path_to_urdf + "_new.urdf"
    with open(path_to_urdf, "w") as file_handle:
        file_handle.write(old_file)

    replace_old_path_urdf(
        path_to_urdf,
        f"package://{urdf_package}",
        package_share_directory + "/",
    )

    return path_to_urdf
