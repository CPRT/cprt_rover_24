import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    navigation_package_dir = get_package_share_directory("navigation")
    config_dir = os.path.join(navigation_package_dir, "config", "elevation_mapping")
    list_params = []
    list_files = [
        "zed2i_robot.yaml",
        "zed2i_elevation_mapping.yaml",
        "zed2i_sensor_processor.yaml",
        # "postprocessing_traversability.yaml",
        "faster_postprocessing_traversability.yaml",
    ]
    for file in list_files:
        if not os.path.isfile(os.path.join(config_dir, file)):
            raise FileNotFoundError("File not found: " + os.path.join(config_dir, file))
        list_params.append(os.path.join(config_dir, file))

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="elevation_mapping",
                executable="elevation_mapping",
                name="elevation_mapping",
                output="screen",
                parameters=list_params,
            ),
        ]
    )
