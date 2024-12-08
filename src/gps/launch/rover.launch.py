import os

import ament_index_python.packages
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("gps"), "config")

    params_file = os.path.join(config_dir, "gps.yaml")

    ublox_remappings = [("fix", "gps/fix"), ("/navheading", "gps/heading")]

    ublox_gps_node = launch_ros.actions.Node(
        package="ublox_gps",
        executable="ublox_gps_node",
        output="both",
        remappings=ublox_remappings,
        parameters=[params_file],
    )

    return launch.LaunchDescription([ublox_gps_node])
