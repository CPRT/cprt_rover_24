import os

import ament_index_python.packages
import launch
import launch_ros.actions
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
        ("imu", "zed/imu_data"),
        ("gps/fix", "gps/fix"),
        ("odometry/filtered", "odometry/filtered/globaousterl"),
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
            ublox_gps_node,
            navsat_node,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=ublox_gps_node,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
