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
                    {"P": 5.0},
                    {"I": 0.0},
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
                    {"P": 5.0},
                    {"I": 0.0},
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
                    {"P": 5.0},
                    {"I": 0.0},
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
                    {"P": 5.0},
                    {"I": 0.0},
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
                package="drive",
                executable="joystick_breakout",
                name="joystick_breakout_node",
            ),
            launch_ros.actions.Node(
                package="drive",
                executable="talon_node",
                name="talon_control_node",
                parameters=[
                    {"wheels": ["frontRight", "frontLeft", "backRight", "backLeft"]},
                    {"max_speed": 1.0},
                    # {'~ticks_per_meter': 4342.2},
                    {"ticks_per_meter": 3354},
                    {"ticks_per_rotation": 4096},
                    {"base_width": 0.9},
                    {"pub_odom": True},
                    {"pub_elec": True},
                    {"stop_movement": False},
                ],
            ),
            launch_ros.actions.Node(
                package="joy", executable="joy_node", name="joystick"
            ),
            # launch_ros.actions.Node(
            #     package="drive",
            #     executable="joystick_drive",
            #     name="joystick_drive_station",
            #     parameters=[
            #         {"PID_max_speed": 1.0},  # m/s
            #         {"PID_max_turn": 1.0},  # rad/s
            #         {"voltage_max_speed": 8.0},  # x/12volts
            #         {"voltage_max_turn": 8.0},  # x/12volts
            #         {"PID": 1},  # PID 1 Voltage 0
            #     ],
            # ),
        ]
    )
