import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription(
  [
        launch_ros.actions.Node(
            package='drive',
            executable='joystick_drive',
            name='joystick_drive_station',
            parameters=[
                {'PID_max_speed': 1.0}, #m/s
                {'PID_max_turn': 1.0}, #rad/s
                {'voltage_max_speed': 8.0}, # x/12volts
                {'voltage_max_turn': 8.0}, # x/12volts
                {'PID': 1}, # PID 1 Voltage 0
            ]
            ),
        launch_ros.actions.Node(
            package='drive',
            executable='joystick_breakout',
            name='joystick_breakout_node'),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joystick'),
        launch_ros.actions.Node(
            package='drive',
            executable='roboclaw_node',
            name='roboclaw_node',
            parameters=[
                {'dev': '/dev/roboclaws/front'},
                {'baud': 115200},
                {'address': 128},
                {'max_speed': 1.0},
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
                {'max_speed': 1.0},
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
                {'max_speed': 1.0},
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
