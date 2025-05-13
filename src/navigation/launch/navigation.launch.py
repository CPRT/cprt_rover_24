import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from pathlib import Path


def generate_launch_description():

    # Get package directory
    pkg_navigation = get_package_share_directory("navigation")

    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    launch_transversability = LaunchConfiguration(
        "launch_transversability", default="True"
    )
    launch_zed = LaunchConfiguration("launch_zed", default="True")

    launch_transversability_cmd = DeclareLaunchArgument(
        "launch_transversability",
        description="Launch elevation mapping pkg if True",
        default_value="True",
    )
    launch_zed_cmd = DeclareLaunchArgument(
        "launch_zed", description="Launch description if True", default_value="True"
    )
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        description="Use simulation (Gazebo) clock if True",
        default_value="false",
    )

    transversability_launch_file_path = (
        Path(pkg_navigation) / "launch" / "traversability_gridmap.launch.py"
    )
    transversability_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(transversability_launch_file_path)),
        condition=IfCondition(launch_transversability),
    )

    svo_path = LaunchConfiguration("svo_path")
    svo_path_launch_arg = DeclareLaunchArgument(
        "svo_path",
        default_value=TextSubstitution(text="live"),
        description="Path to an input SVO file.",
    )

    zed_launch_file_path = Path(pkg_navigation) / "launch" / "zed.launch.py"
    zed_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(zed_launch_file_path)),
        condition=IfCondition(launch_zed),
        launch_arguments={"use_sim_time": use_sim_time, "svo_path": svo_path}.items(),
    )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, "launch", "nav2.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        [
            use_sim_time_cmd,
            launch_transversability_cmd,
            launch_zed_cmd,
            transversability_cmd,
            nav2_cmd,
            svo_path_launch_arg,
            zed_cmd,
        ]
    )
