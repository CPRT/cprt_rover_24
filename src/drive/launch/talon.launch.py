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
    try:
        subprocess.run(["sudo", "enablecan.sh"], check=True)
    except:
        print("enablecan.sh not found")
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
                    {"id": 3},
                    {"P": P},
                    {"I": I},
                    {"D": D},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                    {"sensor_multiplier": 1 / 2760.0},
                    {"max_current": 10.0},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="backLeft",
                parameters=[
                    {"id": 4},
                    {"P": P},
                    {"I": I},
                    {"D": D},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                    {"sensor_multiplier": 1 / 2760.0},
                    {"max_current": 10.0},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="frontRight",
                parameters=[
                    {"id": 1},
                    {"P": P},
                    {"I": I},
                    {"D": D},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                    {"sensor_multiplier": 1 / 2760.0},
                    {"max_current": 10.0},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="backRight",
                parameters=[
                    {"id": 2},
                    {"P": P},
                    {"I": I},
                    {"D": D},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                    {"sensor_multiplier": 1 / 2760.0},
                    {"max_current": 10.0},
                ],
            ),
        ],
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
                    {"angular_slip_ratio": 0.6},
                    {"low_latency_mode": True},
                ],
            ),
        ]
    )
