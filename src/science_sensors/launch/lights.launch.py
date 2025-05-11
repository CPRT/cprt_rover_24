import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="science_sensors",
                executable="pi_gpio_controller",
                name="microscope_light",
                parameters=[
                    {"service_name": "/microscope_light"},
                    {"gpio_pins": [11, 6]},
                ],
            ),
        ]
    )
