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
                name="platform",
                parameters=[
                    {"id": 7},
                    {"P": P},
                    {"I": I},
                    {"D": D},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                    {"watchdog_ms": 200},
                ],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="drill",
                parameters=[
                    {"id": 8},
                    {"P": P},
                    {"I": I},
                    {"D": D},
                    {"max_voltage": 24.0},
                    {"brake_mode": True},
                    {"watchdog_ms": 200},
                ],
            ),

        ],
    )

    return launch.LaunchDescription(
        [
            container,
        ]
    )
