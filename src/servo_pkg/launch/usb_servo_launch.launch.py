import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_servo = get_package_share_directory("servo_pkg")
    parameters_file = os.path.join(pkg_servo, "config", "servo_ranges.yaml")

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="servo_pkg",
                executable="USB_Servo",
                name="USB_Servo_node",
                parameters=[
                    {"serial_port": "/dev/ttyACM0"},
                    {"config_file": parameters_file},
                ],
            ),
            # for example client
            # launch_ros.actions.Node(
            #    package="servo_pkg",
            #    executable="servo_client",
            #    name="servo_client_node",
            # ),
        ]
    )
