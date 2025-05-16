import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import subprocess
import os


def generate_launch_description():
    """Generate launch description with multiple components."""
    try:
        subprocess.run(["sudo", "enablecan.sh"], check=True)
    except Exception:
        print("enablecan.sh not found or failed to run.")

    # Define config path substitution
    config_path = LaunchConfiguration("config_path")

    container = ComposableNodeContainer(
        name="PhoenixContainer",
        namespace="",
        package="ros_phoenix",
        executable="phoenix_container",
        parameters=[config_path],  # Now this is defined correctly
        composable_node_descriptions=[
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="frontLeft",
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="backLeft",
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="frontRight",
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="backRight",
            ),
            ComposableNode(
                package="drive_cpp",
                plugin="TalonDriveController",
                name="talon_drive_controller",
            ),
        ],
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=os.path.join(
                    get_package_share_directory("drive"), "config", "talon_drive.yaml"
                ),
                description="Path to YAML config file",
            ),
            container,
        ]
    )
