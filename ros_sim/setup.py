import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ros_sim"
data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    (os.path.join("share/", package_name), ["package.xml"]),
    (
        os.path.join("share/", package_name, "cfg"),
        [
            "cfg/traj_waypoints.yaml",
        ],
    ),
    (
        os.path.join("share/", package_name, "launch"),
        ["launch/bringup_sim.launch.py"],
    ),
]


def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:
        for path, directories, filenames in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join("share", package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=package_files(data_files, ["resource/"]),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="root@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "ros_sim = ros_sim.pybullet_ex:main",
        ],
    },
)
