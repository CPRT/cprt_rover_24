import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="camera_controller",
                executable="cam_control",
                name="cam_control",
                parameters=[
                    {"port_x": 1},
                    {"port_y": 2},
                    {"min_servo": 0},
                    {"max_servo": 180},
                    {"default_x_pos": 90},
                    {"default_x_pos": 103},
                    {"step_size": 1.0},
                    {"x_axis_index": 1},
                    {"y_axis_index": 0},
                    {"default_button": -1},
                    {"frequency": 10.0},
                ],
            ),
        ]
    )
