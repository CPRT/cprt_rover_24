# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


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

    pkg_localization = get_package_share_directory("localization")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    launch_ouster = LaunchConfiguration("launch_ouster")
    launch_ouster_cmd = DeclareLaunchArgument(
        "launch_ouster",
        default_value="True",
        description="Launch ouster driver if True",
    )

    launch_navsat = LaunchConfiguration("launch_navsat")
    launch_navsat_cmd = DeclareLaunchArgument(
        "launch_navsat", default_value="True", description="Launch navsat node if True"
    )

    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz", default_value="True", description="Launch rviz if True"
    )

    launch_gps = LaunchConfiguration("launch_gps")
    launch_gps_cmd = DeclareLaunchArgument(
        "launch_gps", default_value="True", description="Launch ublox if True"
    )

    rviz_config = os.path.join(
        get_package_share_directory("localization"),
        "config",
        "rviz.views",
        "basicNavMap.rviz",
    )

    rviz_launch_file_path = Path(pkg_localization) / "launch" / "rviz.launch.py"
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(rviz_launch_file_path)]),
        launch_arguments={"rviz_config": rviz_config}.items(),
        condition=IfCondition(launch_rviz),
    )

    gps_launch_file_path = Path(pkg_localization) / "launch" / "gps.launch.py"
    gps_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(gps_launch_file_path)]),
        condition=IfCondition(launch_gps),
    )

    ouster_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "ouster.launch.py")
        )
    )
    navsat_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "navsat.launch.py")
        )
    )

    icp_odometry_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "icp_odometry.launch.py")
        )
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "rtabmap.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    imu_filter_cmd = Node(
        package="localization",
        executable="imu_filter",
        output="log",
        arguments=["--ros-args", "--log-level", "Warn"],
    )

    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "ekf.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_cmd)
    ld.add_action(launch_ouster_cmd)
    ld.add_action(launch_rviz_cmd)
    ld.add_action(launch_gps_cmd)

    if launch_ouster:
        ld.add_action(ouster_cmd)
    if launch_rviz:
        ld.add_action(rviz_cmd)
    if launch_gps:
        ld.add_action(gps_cmd)

    ld.add_action(icp_odometry_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(ekf_cmd)
    ld.add_action(imu_filter_cmd)
    return ld
