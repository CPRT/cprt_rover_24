import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription  
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

launch_args = [
    DeclareLaunchArgument(
        'profile',
        default_value='',
        description='Options are "night", "day", or "testing"'
    ),
    DeclareLaunchArgument(
        'playback_filename',
        default_value='',
        description='When set to a non empty string, plays back the given .svo2 file as if the ZED was connected live.'
    ),
    DeclareLaunchArgument(
        'playback_start_index',
        default_value='0',
        description='When playing back a .svo2 file, start at the given frame index.'
    ),
    DeclareLaunchArgument(
        'record_filename',
        default_value='',
        description='When set to a non empty string, immediately to the given filename. Must have .svo2 suffix.'
    ),
    DeclareLaunchArgument(
        'publish_gl_viewer_data',
        default_value='False',
        description='Options: True or False. When True, publishs additional data for a GLViewer node to display.'
    ),
    DeclareLaunchArgument(
        'publish_6x6_aruco_as_leds',
        default_value='False',
        description="Options: True or False. " + 
                    "When True, aruco tags of the 6X6_50 family will be used to publish on the blue, red and ir topics " +
                    "(ID mapping: 0-15 = blue, 16-30 = red, 31-45 = ir)"
    )
]

def create_zed_node(context):
    loaded_profile_arg = LaunchConfiguration('profile').perform(context)
    loaded_profile_arg = loaded_profile_arg.lower()

    if loaded_profile_arg == "day":
        params = os.path.join(
            get_package_share_directory('camera_processing'),
            'config',
            'day_params.yaml'
        )
    elif loaded_profile_arg == "testing":
        params = os.path.join(
            get_package_share_directory('camera_processing'),
            'config',
            'testing_params.yaml'
        )
    else:
        params = os.path.join(
            get_package_share_directory('camera_processing'),
            'config',
            'night_params.yaml' 
        )

    playback_filename = LaunchConfiguration('playback_filename')
    playback_start_index = LaunchConfiguration('playback_start_index')
    record_filename = LaunchConfiguration('record_filename')
    publish_gl_viewer_data = LaunchConfiguration('publish_gl_viewer_data')
    publish_6x6_aruco_as_leds = LaunchConfiguration('publish_6x6_aruco_as_leds')

    colour_processing_params = os.path.join(
        get_package_share_directory('camera_processing'),
        'config',
        'colour_processing_params.yaml'
    )

    zed_node = Node(
        package="camera_processing",
        executable="zed_node",
        name="zed",
        parameters=[
            params, 
            colour_processing_params,
            {"playback_filename": playback_filename},
            {'record_filename': record_filename},
            {'publish_gl_viewer_data': publish_gl_viewer_data},
            {'publish_6x6_aruco_as_leds': publish_6x6_aruco_as_leds},
            {'playback_start_index': playback_start_index}
        ]
    )

    return [zed_node] 



def generate_launch_description():
    zed_node = OpaqueFunction(function=create_zed_node)

    ld = LaunchDescription(launch_args)

    # Nodes 
    transform_zed_points_node = Node(
        package="camera_processing",
        executable="transform_zed_points_node",
        name="transform_zed_points"
    )

    # Add launch actions
    ld.add_action(zed_node)
    ld.add_action(transform_zed_points_node)
    return ld