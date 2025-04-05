import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="localization",
                executable="imu_pub_node",
                name="localization_node",
                parameters=[
                    {
                        "orientation_covariance": [0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02]
                    },  # orientation_covariance
                    {
                        "angular_velocity_covariance": [
                            0.01,
                            0,
                            0,
                            0,
                            0.01,
                            0,
                            0,
                            0,
                            0.01,
                        ]
                    },  # angular_velocity_covariance
                    {
                        "linear_acceleration_covariance": [
                            0.05,
                            0,
                            0,
                            0,
                            0.05,
                            0,
                            0,
                            0,
                            0.05,
                        ]
                    },  # linear_acceleration_covariance
                    {"Freq": 2.0},  # Publish rate (hz)
                ],
            ),
        ]
    )
