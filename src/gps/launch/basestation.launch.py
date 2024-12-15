import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription(
  [
        launch_ros.actions.Node(
            package='gps',
            executable='rtcm_pub_node',
            name='gps_basestation_node',
            parameters=[
                {'TimingMode': 1}, # Survey In mode
                {'MinTime': 60}, # Survey in time (s)
                {'MinAcc': 10.0}, # Survey In minimum accuracy (m)
                {'Freq': 2.0}, # Publish rate (hz)
                {'Baudrate': 9600},
                {'Device': "/dev/ttyACM0"},
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
