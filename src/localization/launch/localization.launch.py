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
    pkg_localization = get_package_share_directory("localization")

    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="False")
    launch_rviz = LaunchConfiguration("launch_rviz", default="True")
    launch_gps = LaunchConfiguration("launch_gps", default="True")
    launch_desc = LaunchConfiguration("launch_desc", default="True")

    # Launch argument declarations
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", description="Use simulation (Gazebo) clock if True"
    )
    launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz", description="Launch rviz if True"
    )
    launch_gps_cmd = DeclareLaunchArgument(
        "launch_gps", description="Launch ublox if True"
    )
    launch_desc_cmd = DeclareLaunchArgument(
        "launch_desc", description="Launch description if True"
    )

    # RViz launch command
    rviz_config = os.path.join(
        pkg_localization, "config", "rviz.views", "basicNavMap.rviz"
    )
    rviz_launch_file_path = Path(pkg_localization) / "launch" / "rviz.launch.py"
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(rviz_launch_file_path)),
        launch_arguments={"rviz_config": rviz_config}.items(),
        condition=IfCondition(launch_rviz),
    )

    # GPS launch command
    gps_launch_file_path = Path(pkg_localization) / "launch" / "gps.launch.py"
    gps_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gps_launch_file_path)),
        condition=IfCondition(launch_gps),
    )

    # Static description publiser launch command
    desc_launch_file_path = Path(pkg_localization) / "launch" / "description.launch.py"
    desc_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(desc_launch_file_path)),
        condition=IfCondition(launch_desc),
    )

    # Other launch commands
    navsat_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "navsat.launch.py")
        )
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "rtabmap.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "ekf.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Assemble the LaunchDescription and return
    return LaunchDescription(
        [
            use_sim_time_cmd,
            launch_rviz_cmd,
            launch_gps_cmd,
            launch_desc_cmd,
            rviz_cmd,
            gps_cmd,
            navsat_cmd,
            slam_cmd,
            ekf_cmd,
            desc_cmd,
        ]
    )
