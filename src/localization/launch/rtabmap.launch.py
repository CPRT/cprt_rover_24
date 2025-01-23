import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    # Path to the config directory in your custom package
    
    config_dir = os.path.join(get_package_share_directory("my_rtabmap_package"), "config")
    params_file = os.path.join(config_dir, "rtabmap.yaml")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True")

    launch_rtabmapviz = LaunchConfiguration("launch_rtabmapviz")
    launch_rtabmapviz_cmd = DeclareLaunchArgument(
        "launch_rtabmapviz",
        default_value="True",
        description="Whether to launch rtabmapviz")

    # RTAB-Map parameters
    parameters = [{
        "frame_id": "base_link",
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "subscribe_scan_cloud": False,
        "approx_sync": True,
        "publish_tf": True,
        "use_sim_time": use_sim_time,
        "qos_camera_info": 2,

        "Grid/DepthDecimation": "2",
        "Grid/RangeMin": "0.5",
        "Grid/RangeMax": "15.0",
        "Grid/MinClusterSize": "10",
        "Grid/MaxGroundAngle": "30",
        "Grid/NormalK": "10",
        "Grid/CellSize": "0.05",
        "Grid/FlatObstacleDetected": "false",

        "GridGlobal/UpdateError": "0.01",
        "GridGlobal/MinSize": "200",

        "Reg/Strategy": "1"
    }]

    # Remappings for ZED topics
    remappings = [
        ("rgb/image", "zed/zed_node/rgb/image_rect_color"),
        ("rgb/camera_info", "zed/zed_node/rgb/camera_info"),
        ("depth/image", "zed/zed_node/depth/depth_registered"),
        ("odom", "zed/zed_node/odom"),
        ("imu", "zed/zed_node/imu/data"),
        ("goal", "goal_pose"),
        ("map", "map"),
    ]

    static_tf_pub_odom_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    )

    static_tf_pub_base_link_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "zed_left_camera_optical_frame"],
    )

    # Launch Description
    return LaunchDescription([
        # Declare launch arguments
        use_sim_time_cmd,
        launch_rtabmapviz_cmd,
        static_tf_pub_odom_to_base_link,  
        static_tf_pub_base_link_to_camera,
        

        # RTAB-Map Node
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            output="screen",
            parameters=parameters,
            remappings=remappings,
            arguments=["-d", "--delete_db_on_start",
                       "--ros-args", "--log-level", "Warn"]),

        # RTAB-Map Visualization Node (Conditional)
        Node(
            condition=IfCondition(launch_rtabmapviz),
            package="rtabmap_viz",
            executable="rtabmap_viz",
            output="screen",
            parameters=[params_file],
            remappings=remappings),

        # ZED Wrapper Node
        #Node(
         #   package="zed_wrapper",
          #  executable="zed_wrapper_node",
           # name="zed2i",
            #output="screen",
            #parameters=[
             #   os.path.join(os.path.expanduser("~/ros2_ws/src/zed-ros2-wrapper/zed_wrapper/config"), "common_stereo.yaml")
                
            #])
    ])
