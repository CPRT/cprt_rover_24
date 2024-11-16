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
                name="base",
                parameters=[{"id": 1},
                            {"P":5.0},
                            {"I":0.0},
                            {"D":0.0},
                            {"max_voltage": 12.0},
                            {"invert_sensor":True}],
                
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="diff1",
                parameters=[{"id": 2},
                            {"P":100.0},
                            {"I":0.0},
                            {"D":0.0}],
                
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="diff2",
                parameters=[{"id": 3},
                            {"P":100.0},
                            {"I":0.0},
                            {"D":0.0}],
                
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="elbow",
                parameters=[{"id": 4},
                            {"P":100.0},
                            {"I":0.0},
                            {"D":0.0}],
                
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="wristTilt",
                parameters=[{"id": 5},
                            {"P":5.0},
                            {"I":0.0},
                            {"D":0.0},
                            {"max_voltage": 6.0},
                            {"invert_sensor":True}],
                
            ),
            ComposableNode(
                package="ros_phoenix",
                plugin="ros_phoenix::TalonSRX",
                name="wristTurn",
                parameters=[{"id": 6},
                            {"P":5.0},
                            {"I":0.0},
                            {"D":0.0},
                            {"max_voltage": 6.0},
                            {"invert_sensor":True}],
                
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription([container,
        launch_ros.actions.Node(
            package='arm_interface',
            executable='trajectory_interpreter',
            name='trajectory_interpreter_node'),
        launch_ros.actions.Node(
            package='arm_interface',
            executable='joystick_arm_controller',
            name='joystick_arm_controller_node'),            
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joystick'),
        launch_ros.actions.Node(
            package='drive',
            executable='joystick_breakout',
            name='joystick_breakout_node'),
            
            ])
