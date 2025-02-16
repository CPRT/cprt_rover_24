import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from pathlib import Path
from launch.conditions import IfCondition


def generate_launch_description():

    pkg_drive = get_package_share_directory("drive_cpp")
    pkg_control = get_package_share_directory("joystick_control")

    launch_joy = LaunchConfiguration("launch_joy")
    launch_joy_cmd = DeclareLaunchArgument(
        "launch_joy",
        default_value="False",
        description="Launch joy node if True",
    )

    parameters_file = os.path.join(pkg_control, "config", "thrustmaster.yaml")

    return LaunchDescription(
        [
            launch_joy_cmd,
            Node(
                package="joystick_control",
                executable="thrustmaster_control",
                name="thrustmaster_control",
                output="screen",
                parameters=[parameters_file],
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                condition=IfCondition(launch_joy),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_drive, "launch", "talonDrive.launch.py")
                ),
            ),
        ]
    )
