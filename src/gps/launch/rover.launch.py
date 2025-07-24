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

    rover_config_file = os.path.join(config_dir, "Rover_config.ubx")
    rover_gps = (
        "ubxload --port /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00 --baudrate 9600 --infile " + rover_config_file
    )
    os.system(rover_gps)

    ublox_remappings = [("fix", "gps/fix"), ("/navheading", "gps/heading")]

    ublox_gps_node = launch_ros.actions.Node(
        package="ublox_gps",
        executable="ublox_gps_node",
        output="both",
        remappings=ublox_remappings,
        parameters=[params_file],
    )

    return launch.LaunchDescription([ublox_gps_node])
