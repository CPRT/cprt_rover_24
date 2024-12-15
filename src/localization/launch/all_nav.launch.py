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


def generate_launch_description():

    pkg_localization = get_package_share_directory("localization")
    pkg_navigation = get_package_share_directory("navigation")
    pkg_description = get_package_share_directory("description")
    pkg_drive = get_package_share_directory("drive")
    pkg_ouster = get_package_share_directory("ouster_ros")
    pkg_camera_processing = get_package_share_directory("camera_processing")

    localization_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, "launch", "localization.launch.py")
        )
    )

    ouster_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ouster, "launch", "driver.launch.py")
        )
    )

    description_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, "launch", "robot_state_publisher.launch.py")
        )
    )

    drive_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drive, "launch", "roverdrivewithoutjoystick.launch.py")
        )
    )

    zed_node_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_camera_processing, "launch", "zed.launch.py")
        ),
        launch_arguments={"profile": "night"}.items(),
    )

    ld = LaunchDescription()

    # ld.add_action(nav_launch_cmd)
    ld.add_action(localization_launch_cmd)
    ld.add_action(ouster_launch_cmd)
    ld.add_action(description_launch_cmd)
    ld.add_action(drive_launch_cmd)
    ld.add_action(zed_node_launch_cmd)

    return ld
