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


from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    local_params_file = os.path.join(
        get_package_share_directory("localization"), "config", "ekf_local.yaml"
    )
    global_params_file = os.path.join(
        get_package_share_directory("localization"), "config", "ekf_global.yaml"
    )

    param_substitutions = {"use_sim_time": use_sim_time}

    configured_local_params = RewrittenYaml(
        source_file=local_params_file,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    configured_global_params = RewrittenYaml(
        source_file=global_params_file,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    local_ekf_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="local_ekf",
        output="log",
        parameters=[configured_local_params],
        remappings=[
            ("odometry/filtered", "odometry/filtered/local"),
            ("accel/filtered", "/accel/local"),
        ],
    )
    global_ekf_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="global_ekf",
        output="log",
        parameters=[configured_global_params],
        remappings=[
            ("odometry/filtered", "odometry/filtered/global"),
            ("accel/filtered", "/accel/global"),
        ],
    )

    return LaunchDescription([use_sim_time_cmd, local_ekf_cmd, global_ekf_cmd])
