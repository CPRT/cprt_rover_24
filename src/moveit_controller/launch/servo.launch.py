from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_param_builder import ParameterBuilder

import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro
from moveit_configs_utils import MoveItConfigsBuilder

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def test_launch(moveit_config, launch_package_path=None):
    """
    Launches a self contained demo

    launch_package_path is optional to use different launch and config packages

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
    """
    if launch_package_path == None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    # Get parameters for the Servo node
    """servo_params = {
        "moveit_servo":ParameterBuilder("arm_srdf3")
        .yaml(
            file_path="config/arm_config.yaml",
        )
        .to_dict()
    }"""

    # Get parameters for the Servo node
    servo_yaml = load_yaml("arm_srdf3", "config/arm_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    ld.add_action(
        # Launch a standalone Servo node.
        # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            parameters=[
                servo_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {"use_intra_process_comms": True},
            ],
            output="screen",
        )
    )
    """
    
    container = ComposableNodeContainer(
       name="moveit_servo_demo_container",
       namespace="/",
       package="rclcpp_components",
       executable="component_container_mt",
       composable_node_descriptions=[
           # Example of launching Servo as a node component
           # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
           ComposableNode(
               package="moveit_servo",
               plugin="moveit_servo::ServoNode",
               name="servo_node",
               parameters=[
                   servo_params,
                   moveit_config.robot_description,
                   moveit_config.robot_description_semantic,
               ],
               extra_arguments=[{'use_intra_process_comms' : True}]
           ),
       ],
       output="screen",
    )
    
    ld.add_action(container)"""

    return ld


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "arm_urdf3", package_name="arm_srdf3"
    ).to_moveit_configs()
    return test_launch(moveit_config)
