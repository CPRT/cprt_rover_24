import launch
import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import subprocess

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
                name="act1",
                parameters=[{"id": 11}, {"P": 5.0}, {"I": 0.0}, {"D": 0.0}],
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="act2",
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
                package="servo_pkg",
                executable="USB_Servo",
                name="USB_Servo_node",
            ),
        ]
    )