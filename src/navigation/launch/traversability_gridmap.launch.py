import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    navigation_package_dir = get_package_share_directory("navigation")
    config_dir = os.path.join(navigation_package_dir, "config", "elevation_mapping")
    list_params = []
    list_files = [
        "zed2i_robot.yaml",
        "zed2i_elevation_mapping.yaml",
        "zed2i_sensor_processor.yaml",
        "postprocessing_traversability.yaml",
    ]
    for file in list_files:
        if not os.path.isfile(os.path.join(config_dir, file)):
            raise FileNotFoundError("File not found: " + os.path.join(config_dir, file))
        list_params.append(os.path.join(config_dir, file))

    node = ComposableNode(
        package="elevation_mapping",
        plugin="elevation_mapping::ElevationMapNode",
        name="elevation_mapping",
        parameters=list_params,
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    return launch.LaunchDescription(
        [
            LoadComposableNodes(
                composable_node_descriptions=[node],
                target_container="/zed/zed_container",
            )
        ]
    )
