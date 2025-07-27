import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="science_sensors",
                executable="geiger_counter",
                name="geiger_counter",
            ),
        ]
    )
