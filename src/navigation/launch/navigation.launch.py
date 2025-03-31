import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from pathlib import Path


def generate_launch_description():
    # Get package directory
    pkg_navigation = get_package_share_directory("navigation")

    # Declare launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_transversability = LaunchConfiguration("launch_transversability")
    launch_zed = LaunchConfiguration("launch_zed")
    zed_fuse_gps = LaunchConfiguration("zed_fuse_gps")

    # Create LaunchDescription object
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true"
    ))
    
    ld.add_action(DeclareLaunchArgument(
        "launch_transversability",
        default_value="true",
        description="Launch elevation mapping pkg if true"
    ))
    
    ld.add_action(DeclareLaunchArgument(
        "launch_zed",
        default_value="true",
        description="Launch ZED camera if true"
    ))
    
    ld.add_action(DeclareLaunchArgument(
        "zed_fuse_gps",
        default_value="true",
        description="Use false here if indoors, when true it waits for a single fix before giving a pose estimate."
    ))

    # Include transversability launch file
    transversability_launch_file_path = (
        Path(pkg_navigation) / "launch" / "traversability_gridmap.launch.py"
    )
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(transversability_launch_file_path)),
        condition=IfCondition(launch_transversability),
    ))

    # Include ZED launch file
    zed_launch_file_path = Path(pkg_navigation) / "launch" / "zed.launch.py"
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(zed_launch_file_path)),
        condition=IfCondition(launch_zed),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "zed_fuse_gps": zed_fuse_gps
        }.items(),
    ))

    # Include nav2 launch file
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "nav2.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    ))

    return ld