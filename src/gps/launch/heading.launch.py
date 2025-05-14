import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("gps"), "config")

    Heading_config_file = os.path.join(config_dir, "Heading_config.ubx")
    heading = (
        "ubxload --port /dev/ttyUSB0 --baudrate 115200 --infile " + Heading_config_file
    )
    # os.system(heading)

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="gps",
                executable="heading_pub_node",
                name="gps_heading_node",
                parameters=[
                    {"frame_id": "gps_link"},
                    {"Freq": 5.0},  # Publish rate (hz)
                    {"Baudrate": 115200},
                    {"Device": "/dev/ttyUSB0"},
                ],
                remappings=[("heading", "gps/heading")],
            ),
        ]
    )
