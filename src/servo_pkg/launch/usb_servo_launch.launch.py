import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="servo_pkg",
                executable="USB_Servo",
                name="USB_Servo_node",
            ),
        ]
    )
