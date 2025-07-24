import os

import ament_index_python.packages
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from serial import Serial
from pyubx2 import SET, UBX_PROTOCOL, UBXMessage, UBXReader


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("gps"), "config")
    with Serial("/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00", 115200) as stream:
        msg = UBXMessage(
            "CFG",
            "CFG-CFG",
            SET,
            clearMask=b"\x1f\x1f\x00\x00",  # clear everything
            loadMask=b"\x1f\x1f\x00\x00",  # reload everything
            devBBR=1,  # clear from battery-backed RAM
            devFlash=1,  # clear from flash memory
            devEEPROM=1,  # clear from EEPROM memory
        )
        stream.write(msg.serialize)
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
