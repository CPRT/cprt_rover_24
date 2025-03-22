import launch
import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import subprocess

P = 2.00
I = 0.000002
D = 0.0000000001

# values for setting input_type
ANALOG = 1
ABSOLUTE = 2
RELATIVE = 3


def generate_launch_description():
    """Generate launch description with multiple components."""
    subprocess.run(["sudo", "enablecan.sh"], check=True)
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
                    {"id": 8},
                    {"P": P},
                    {"I": I},
                    {"D": D},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                    {"input_type": ABSOLUTE},
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
                    {"wheels": ["frontLeft"]},
                    {"max_speed": 2.0},
                    {"base_width": 0.9},
                    {"pub_odom": True},
                    {"pub_elec": True},
                    {"wheel_rad": 0.10},
                ],
            ),
        ]
    )
