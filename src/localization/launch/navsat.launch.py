from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True")

    navsat_parameters = [{
        "frequency": 10.0,
        "delay": 0.0,
        "magnetic_declination_radians": 0.20944, # 0.226893 for Drumheller, 0.20944 for Ottawa
        "yaw_offset": 1.570796327,  # IMU reads 0 facing magnetic north, not east
        "zero_altitude": True,
        "broadcast_utm_transform": False,
        "publish_filtered_gps": False,
        "use_odometry_yaw": False,
        "wait_for_datum": False
    }]

    navsat_remappings = [
        ("imu/data", "imu/from/gps"),
        ("gps/fix", "gps/fix"),
        ("odometry/filtered", "odometry/filtered/global")
    ]

    return LaunchDescription([

        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            output="log",
            parameters=[navsat_parameters],
            remappings=navsat_remappings,
            arguments=["--ros-args", "--log-level", "Warn"]
        )
    ])