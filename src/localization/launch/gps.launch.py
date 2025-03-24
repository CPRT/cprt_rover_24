import os

import ament_index_python.packages
import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg_gps = get_package_share_directory("gps")

    config_dir = os.path.join(get_package_share_directory("localization"), "config")

    params_file = os.path.join(config_dir, "navsat.yaml")

    ublox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gps, "launch", "rover.launch.py")
        )
    )

    navsat_remappings = [
        ("imu", "gps/heading"),
        ("gps/fix", "gps/fix"),
        ("odometry/filtered", "odometry/filtered/global"),
        ("odometry/gps", "gps/odom"),
    ]

    navsat_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="navsat_transform_node",
        output="log",
        parameters=[params_file],
        remappings=navsat_remappings,
        arguments=["--ros-args", "--log-level", "Warn"],
    )

    return launch.LaunchDescription(
        [
            navsat_node,
            ublox_cmd,
        ]
    )
