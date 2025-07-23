import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    # Get the share directory of your package
    costmap_seeding_share_dir = get_package_share_directory("costmap_seeding")

    # Define the path to the parameters YAML file
    # We assume the YAML file is located in a 'config' subdirectory within your package's share directory
    param_file_path = os.path.join(
        costmap_seeding_share_dir, "config", "can_elevation_downloader_params.yaml"
    )

    # Declare a launch argument for the parameters file, allowing it to be overridden
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=TextSubstitution(text=param_file_path),
        description="Path to the YAML file with node parameters",
    )

    # Define the node
    can_elevation_downloader_node = Node(
        package="costmap_seeding",
        executable="DownloadCanElevationMaps",  # This should match the entry point in setup.py
        name="can_elevation_downloader_node",  # This should match the node name in the C++ or Python code
        parameters=[
            LaunchConfiguration("params_file")
        ],  # Load parameters from the specified YAML file
    )

    # Create the launch description and add the actions
    return LaunchDescription([declare_params_file_cmd, can_elevation_downloader_node])
