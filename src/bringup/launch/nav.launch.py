from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
import os


def get_included_launch_descriptions(launch_files):
    included_launches = []
    for pkg, file in launch_files:
        pkg_share = FindPackageShare(pkg).find(pkg)
        file_path = os.path.join(pkg_share, "launch", file)
        included_launches.append(
            IncludeLaunchDescription(PythonLaunchDescriptionSource(file_path))
        )
    return included_launches


def generate_launch_description():
    SetEnvironmentVariable('ROS_LOG_LEVEL', 'info'),
    launch_files = [
        ("drive", "talon.launch.py"),
        ("camera_streaming", "webRTC.launch.py"),
        ("joystick_control", "controller.launch.py"),
        ("navigation", "navigation.launch.py"),
        ("localization", "localization.launch.py"),
        ("nav_commanders", "gps_commander.launch.py"),
        ("servo_pkg", "usb_servo_launch.launch.py"),
        # TODO: add lights node
    ]
    return LaunchDescription(get_included_launch_descriptions(launch_files))
