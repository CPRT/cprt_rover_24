import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("node_manager"), "config")

    params_file = os.path.join(config_dir, "launch_files.yaml")

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="node_manager",
                executable="node_manager",
                output="log",
                parameters=[params_file],
            )
        ]
    )
