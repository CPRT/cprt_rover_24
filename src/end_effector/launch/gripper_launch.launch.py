import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="end_effector",
                executable="gripper",
                name="gripper_node",
            ),
            launch_ros.actions.Node(
                package="joy", executable="joy_node", name="joystick"
            ),
        ]
    )