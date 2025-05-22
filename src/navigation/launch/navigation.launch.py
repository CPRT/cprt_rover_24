import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition


def generate_launch_description():
    # Get package directories
    pkg_navigation = get_package_share_directory("navigation")

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    svo_path = LaunchConfiguration("svo_path")
    launch_transversability = LaunchConfiguration("launch_transversability")
    launch_zed = LaunchConfiguration("launch_zed")

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    declare_transversability = DeclareLaunchArgument(
        "launch_transversability", default_value="True"
    )
    declare_zed = DeclareLaunchArgument("launch_zed", default_value="True")
    declare_svo_path = DeclareLaunchArgument(
        "svo_path",
        default_value=TextSubstitution(text="live"),
        description="Path to an input SVO file.",
    )

    # Include nav2 (with composition and container name)
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "nav2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_composition": "False",
        }.items(),
    )

    # Include traversability mapping if enabled
    transversability_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "traversability_gridmap.launch.py")
        ),
        condition=IfCondition(launch_transversability),
    )

    # Include ZED launch if enabled
    zed_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "zed.launch.py")
        ),
        condition=IfCondition(launch_zed),
        launch_arguments={"use_sim_time": use_sim_time, "svo_path": svo_path}.items(),
    )
    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_svo_path,
            declare_transversability,
            declare_zed,
            nav2_cmd,
            transversability_cmd,
            zed_cmd,
        ]
    )
