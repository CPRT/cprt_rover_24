import launch
import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import subprocess

P = 2.00
I = 0.000002
D = 0.0000000001


def generate_launch_description():
    """Generate launch description with multiple components."""
    # disable enablecan.sh when testing not on jetson
    # subprocess.run(["sudo", "enablecan.sh"], check=True)
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
                    {"P": P},
                    {"I": I},
                    {"D": D},
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
                    {"P": P},
                    {"I": I},
                    {"D": D},
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
                    {"P": P},
                    {"I": I},
                    {"D": D},
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
                    {"P": P},
                    {"I": I},
                    {"D": D},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="base",
                parameters=[
                    {"id": 10},
                    {"P": 5.0},
                    {"I": 0.0},
                    {"D": 0.0},
                    {"max_voltage": 12.0},
                    {"invert_sensor": True},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="diff1",
                parameters=[{"id": 11}, {"P": 5.0}, {"I": 0.0}, {"D": 0.0}],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="diff2",
                parameters=[{"id": 12}, {"P": 5.0}, {"I": 0.0}, {"D": 0.0}],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="elbow",
                parameters=[{"id": 13}, {"P": 5.0}, {"I": 0.0}, {"D": 0.0}],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="wristTilt",
                parameters=[
                    {"id": 14},
                    {"P": 5.0},
                    {"I": 0.0},
                    {"D": 0.0},
                    {"max_voltage": 6.0},
                    {"invert_sensor": True},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="wristTurn",
                parameters=[
                    {"id": 15},
                    {"P": 5.0},
                    {"I": 0.0},
                    {"D": 0.0},
                    {"max_voltage": 6.0},
                    {"invert_sensor": True},
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
                    {"max_speed": 2.0},
                    {"base_width": 0.9},
                    {"pub_odom": True},
                    {"pub_elec": True},
                    {"wheel_rad": 0.10},
                ],
            ),
        ]
    )
