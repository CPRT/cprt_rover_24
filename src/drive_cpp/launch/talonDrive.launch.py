import launch
import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""

    container = ComposableNodeContainer(
        name="PhoenixContainer",
        namespace="",
        package="ros_phoenix",
        executable="phoenix_container",
        parameters=[{"interface": "can0"}],
        composable_node_descriptions=[
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="frontLeft",
                parameters=[
                    {"id": 1},
                    {"P": 2.0},
                    {"I": 0.012},
                    {"D": 0.0},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="backLeft",
                parameters=[
                    {"id": 2},
                    {"P": 2.0},
                    {"I": 0.012},
                    {"D": 0.0},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="frontRight",
                parameters=[
                    {"id": 3},
                    {"P": 2.0},
                    {"I": 0.012},
                    {"D": 0.0},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="backRight",
                parameters=[
                    {"id": 4},
                    {"P": 2.0},
                    {"I": 0.012},
                    {"D": 0.0},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                ],
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            container,
            launch_ros.actions.Node(
                package="drive_cpp",
                executable="drive_cpp",
                name="talon_control_node",
                parameters=[
                    {"wheels": ["frontRight", "frontLeft", "backRight", "backLeft"]},
                    {"max_speed": 1.0},
                    {"base_width": 0.9},
                    {"pub_odom": True},
                    {"pub_elec": True},
                    {"wheel_rad": 0.105},
                ],
            ),
        ]
    )
