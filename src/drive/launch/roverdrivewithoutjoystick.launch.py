import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription(
  [
        launch_ros.actions.Node(
            package='drive',
            executable='roboclaw_node',
            name='roboclaw_node',
            parameters=[
                {'dev': '/dev/roboclaws/front'},
                {'baud': 115200},
                {'address': 128},
                {'max_speed': 2.0},
                {'ticks_per_meter': 3354},
                {'ticks_per_rotation': 4096},
                {'base_width': 0.81},
                {'pub_odom': True},
                {'pub_elec': True},
                {'stop_movement': False},
            ]
        ),
        launch_ros.actions.Node(
            package='drive',
            executable='roboclaw_node',
            name='roboclaw_node',
            parameters=[
                {'dev': '/dev/roboclaws/mid'},
                {'baud': 115200},
                {'address': 128},
                {'max_speed': 2.0},
                {'ticks_per_meter': 3354},
                {'ticks_per_rotation': 4096},
                {'base_width': 0.81},
                {'pub_odom': True},
                {'pub_elec': True},
                {'stop_movement': False},
            ]
        ),
        launch_ros.actions.Node(
            package='drive',
            executable='roboclaw_node',
            name='roboclaw_node',
            parameters=[
                {'dev': '/dev/roboclaws/back'},
                {'baud': 115200},
                {'address': 128},
                {'max_speed': 2.0},
                {'ticks_per_meter': 3354},
                {'ticks_per_rotation': 4096},
                {'base_width': 0.81},
                {'pub_odom': True},
                {'pub_elec': True},
                {'stop_movement': False},
            ]
        )
  ]
  )
