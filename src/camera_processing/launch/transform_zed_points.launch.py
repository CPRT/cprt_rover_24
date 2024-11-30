from launch import LaunchDescription 
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Nodes 
    transform_zed_points_node = Node(
        package="camera_processing",
        executable="transform_zed_points_node",
        name="transform_zed_points"
    )

    # Add launch actions
    ld.add_action(transform_zed_points_node)
    return ld