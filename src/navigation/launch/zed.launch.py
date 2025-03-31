import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    zed_launch_filepath = os.path.join(
        get_package_share_directory("zed_wrapper"), "launch", "zed_camera.launch.py"
    )
    zed_override_params_filepath = os.path.join(
        get_package_share_directory("navigation"),
        "config",
        "zed",
        "zed_override_params.yaml",
    )

    # Launch configuration variables
    svo_path = LaunchConfiguration("svo_path")
    zed_fuse_gps = LaunchConfiguration("zed_fuse_gps")
    use_zed_efk = LaunchConfiguration("use_zed_efk")

    # Launch argument declarations
    svo_path_launch_arg = DeclareLaunchArgument(
        "svo_path",
        default_value=TextSubstitution(text="live"),
        description="Path to an input SVO file.",
    )
    fuse_gps_arg = DeclareLaunchArgument(
        "zed_fuse_gps",
        default_value="true",
        description="Use false here if indoors, when true it waits for a single fix before giving a pose estimate."
    )
    use_zed_efk_arg = DeclareLaunchArgument(
        "use_zed_efk",
        default_value="true",
        description="Publishes odom and map"
    )

    zed_launch_arguments = {
        "ros_params_override_path": zed_override_params_filepath,
        "camera_model": "zed2i",
        "publish_urdf": "true",
        "publish_tf": use_zed_efk,
        "publish_map_tf": use_zed_efk,
        "gnss_fusion_enabled": zed_fuse_gps,
        "svo_path": svo_path,
    }

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_filepath),
        launch_arguments=zed_launch_arguments.items(),
    )

    ld = LaunchDescription()
    ld.add_action(svo_path_launch_arg)
    ld.add_action(fuse_gps_arg)
    ld.add_action(use_zed_efk_arg)
    ld.add_action(zed_launch)
    
    return ld