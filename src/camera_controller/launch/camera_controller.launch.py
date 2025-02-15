import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="camera_controller",
                executable="cam_control",
                name="gas_sensor",
                parameters=[
                    {"port_x": 0},
                    {"port_y": 1},
                    {"min_servo": 0},
                    {"max_servo": 180},
                    {"default_x": 90},
                    {"default_y": 90},
                    {"step_size": 1.0},
                    {"x_axis_index": 1},
                    {"y_axis_index": 0},
                    {"default_button": -1},
                    {"frequency": 10.0},
                ],
            ),
        ]
    )
