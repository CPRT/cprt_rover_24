# Copyright 2023 Ouster, Inc.
#

"""Launch ouster nodes using a composite container"""

from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """
    Generate launch description for running ouster_ros components in a single
    process/container.
    """
    ouster_ros_pkg_dir = get_package_share_directory("ouster_ros")

    # Set params_file directly as requested, no longer a launch argument
    params_file = os.path.join(
        get_package_share_directory("navigation"),
        "config",
        "ouster",
        "driver_params.yaml",
    )

    ouster_ns = LaunchConfiguration("ouster_ns")
    ouster_ns_arg = DeclareLaunchArgument("ouster_ns", default_value="ouster")

    rviz_enable = LaunchConfiguration("viz")
    rviz_enable_arg = DeclareLaunchArgument("viz", default_value="False")

    auto_start = LaunchConfiguration("auto_start")
    auto_start_arg = DeclareLaunchArgument("auto_start", default_value="True")

    # Define Composable Nodes for Ouster
    os_sensor = ComposableNode(
        package="ouster_ros",
        plugin="ouster_ros::OusterSensor",
        name="os_sensor",
        namespace=ouster_ns,
        parameters=[params_file, {"auto_start": auto_start}],
    )

    os_cloud = ComposableNode(
        package="ouster_ros",
        plugin="ouster_ros::OusterCloud",
        name="os_cloud",
        namespace=ouster_ns,
        parameters=[params_file],
    )

    os_image = ComposableNode(
        package="ouster_ros",
        plugin="ouster_ros::OusterImage",
        name="os_image",
        namespace=ouster_ns,
        parameters=[params_file],
    )

    # Load into an existing container
    load_ouster_nodes_into_zed_container = LoadComposableNodes(
        composable_node_descriptions=[os_sensor, os_cloud, os_image],
        target_container="/zed/zed_container",
    )

    # RViz Launch file inclusion
    rviz_launch_file_path = Path(ouster_ros_pkg_dir) / "launch" / "rviz.launch.py"
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(rviz_launch_file_path)]),
        condition=IfCondition(rviz_enable),
    )

    return launch.LaunchDescription(
        [
            ouster_ns_arg,
            rviz_enable_arg,
            auto_start_arg,
            rviz_launch,
            LogInfo(msg="Waiting to compose Ouster nodes into /zed/zed_container"),
            load_ouster_nodes_into_zed_container,
        ]
    )
