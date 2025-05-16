import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    config_path = LaunchConfiguration("config_path")
    target_container = LaunchConfiguration("target_container")

    # Define each TalonSRX node explicitly
    front_left = ComposableNode(
        package="ros_phoenix",
        plugin="ros_phoenix::TalonSRX",
        name="frontLeft",
        parameters=[config_path],
    )

    back_left = ComposableNode(
        package="ros_phoenix",
        plugin="ros_phoenix::TalonSRX",
        name="backLeft",
        parameters=[config_path],
    )

    front_right = ComposableNode(
        package="ros_phoenix",
        plugin="ros_phoenix::TalonSRX",
        name="frontRight",
        parameters=[config_path],
    )

    back_right = ComposableNode(
        package="ros_phoenix",
        plugin="ros_phoenix::TalonSRX",
        name="backRight",
        parameters=[config_path],
    )

    # Define the drive controller node
    drive_controller = ComposableNode(
        package="drive_cpp",
        plugin="TalonDriveController",
        name="talon_drive_controller",
        parameters=[config_path],
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "target_container",
            default_value="/random_invalid_container",
            description="Name of the target container to load Talon nodes into",
        ),
        DeclareLaunchArgument(
            "config_path",
            default_value=os.path.join(
                get_package_share_directory("drive"),
                "config",
                "talon_drive.yaml"
            ),
            description="Path to parameter YAML file",
        ),
        LoadComposableNodes(
            composable_node_descriptions=[
                front_left,
                back_left,
                front_right,
                back_right,
                drive_controller,
            ],
            target_container=target_container,
        ),
    ])
