from launch import LaunchDescription  
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # LaunchConfigurations
    trail = LaunchConfiguration('trail')
    trail_args = DeclareLaunchArgument(
        'trail',
        default_value='error2',
        description='The type of trail to follow. Options: blue, red, ir'
    )

    # Nodes
    node = Node(
        package="nav_commanders",
        executable="circ_led_trails",
        name="circ_led_trails",
        parameters=[
            {'trail': trail},
        ]
    )

    ld.add_action(trail_args)
    ld.add_action(node)
    return ld