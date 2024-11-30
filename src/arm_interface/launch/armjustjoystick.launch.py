import launch
import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""


    return launch.LaunchDescription([
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