import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition


# Make launch configeration, defult false, launch talon on rover. Defult True for xbox
def generate_launch_description():
    """Generate launch description with multiple components."""
    pkg_drive = get_package_share_directory("drive")
    launch_backend = LaunchConfiguration("launch_backend", default="False")
    launch_backend_cmd = DeclareLaunchArgument(
        "launch_backend",
        default_value="False",
        description="Launch the motor controller if True",
    )

    backend_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_drive, "launch", "talon.launch.py")
        ),
        condition=IfCondition(launch_backend),
    )

    return launch.LaunchDescription(
        [
            launch_backend_cmd,
            backend_cmd,
            launch_ros.actions.Node(
                package="joy", executable="joy_node", name="joystick"
            ),
            launch_ros.actions.Node(
                package="drive",
                executable="joystick_controller",
                name="joystick_controller",
                parameters=[
                    {"linear_axis_index": 1},
                    {"turn_axis_index": 0},
                    {"max_linear_speed": 2.0},
                ],
            ),
        ]
    )
