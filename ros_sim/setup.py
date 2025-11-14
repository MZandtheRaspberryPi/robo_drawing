import os
from setuptools import find_packages, setup

package_name = "ros_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
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
    ],
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
