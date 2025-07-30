import launch
import launch_ros.actions


def generate_launch_description():
    parent_params = os.path.join(pkg_servo, "config", "parent_config.yaml")

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="servo_pkg",
                executable="i2c_Servo",
                name="USB_Servo_node",
                parameters=[parent_params],
            ),
            launch_ros.actions.Node(
                package="servo_pkg", executable="servo_client", name="servo_client_node"
            ),
        ]
    )
