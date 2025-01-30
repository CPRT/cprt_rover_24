import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():
    # Path to the config directory in your custom package

    config_dir = os.path.join(get_package_share_directory("localization"), "config")
    params_file = os.path.join(config_dir, "rtabmap.yaml")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True",
    )

    launch_rtabmapviz = LaunchConfiguration("launch_rtabmapviz")
    launch_rtabmapviz_cmd = DeclareLaunchArgument(
        "launch_rtabmapviz",
        default_value="False",
        description="Whether to launch rtabmapviz",
    )

    # RTAB-Map parameters
    parameters = [
        {
            "frame_id": "base_link",
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_scan_cloud": False,
            "approx_sync": True,
            "publish_tf": False,
            "use_sim_time": use_sim_time,
            "qos_camera_info": 2,
            "Grid/DepthDecimation": "2",
            "Grid/RangeMin": "0.5",
            "Grid/RangeMax": "10.0",
            "Grid/MinClusterSize": "10",
            "Grid/MaxGroundAngle": "30",
            "Grid/NormalK": "10",
            "Grid/CellSize": "0.05",
            "Grid/FlatObstacleDetected": "false",
            "GridGlobal/UpdateError": "0.01",
            "GridGlobal/MinSize": "200",
            "Reg/Strategy": "1",
        }
    ]

    # Remappings for ZED topics
    remappings = [
        ("rgb/image", "zed/zed_node/rgb/image_rect_color"),
        ("rgb/camera_info", "zed/zed_node/rgb/camera_info"),
        ("depth/image", "zed/zed_node/depth/depth_registered"),
        ("odom", "odometry/filtered/local"),
        ("imu", "zed/zed_node/imu/data"),
        ("map", "map"),
    ]

    # Launch Description
    return LaunchDescription(
        [
            # Declare launch arguments
            use_sim_time_cmd,
            launch_rtabmapviz_cmd,
            # RTAB-Map Node
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=parameters,
                remappings=remappings,
                arguments=[
                    "-d",
                    "--delete_db_on_start",
                    "--ros-args",
                    "--log-level",
                    "Warn",
                ],
            ),
            Node(
                condition=IfCondition(launch_rtabmapviz_cmd),
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=[params_file],
                remappings=remappings,
            ),
        ]
    )
