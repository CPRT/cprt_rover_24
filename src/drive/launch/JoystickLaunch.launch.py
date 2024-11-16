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
                {'PID_max_speed': 2.0}, #m/s
                {'PID_max_turn': 2.0}, #rad/s
                {'voltage_max_speed': 8.0}, # x/12volts
                {'voltage_max_turn': 8.0}, # x/12volts
                {'PID': 1}, # PID 1 Voltage 0
            ]
            ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joystick'),
        launch_ros.actions.Node(
            package='drive',
            executable='joystick_breakout',
            name='joystick_breakout_node'),
  ]
  )
