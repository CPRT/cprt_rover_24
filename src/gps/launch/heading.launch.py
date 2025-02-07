import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="gps",
                executable="heading_pub_node",
                name="gps_heading_node",
                parameters=[
                    {"frame_id": "gps"},
                    {"Freq": 5.0},  # Publish rate (hz)
                    {"Baudrate": 115200},
                    {"Device": "/dev/ttyUSB0"},
                ],
            ),
        ]
    )
