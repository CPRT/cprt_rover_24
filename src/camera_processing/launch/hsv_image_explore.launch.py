import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription  
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # LaunchConfigurations
    pipeline_name = LaunchConfiguration('pipeline')
    pipeline_name_arg = DeclareLaunchArgument(
        'pipeline',
        default_value='blue',
        description='Options are blue, red, and ir.'
    )

    colour_processing_params = os.path.join(
        get_package_share_directory('camera_processing'),
        'config',
        'colour_processing_params.yaml'
    )

    # Nodes
    node = Node(
        package="camera_processing",
        executable="hsv_image_explore",
        name="hsv_image_explore",
        parameters=[
            {'pipeline': pipeline_name},
            colour_processing_params]
    )

    # Add launch actions
    ld.add_action(pipeline_name_arg)
    ld.add_action(node)
    return ld