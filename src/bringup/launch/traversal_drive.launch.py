import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Launches:
    1. The 'lights' node from the 'gpio_controller' package.
    2. The 'talon.launch.py' launch file from the 'drive' package.
    """

    # 1. Define the 'lights' node
    # This assumes 'gpio_controller' is the package name and 'lights' is the executable
    # as defined by your setup.py entry point: "lights = gpio_controller.lights:main"
    lights_node = Node(
        package="gpio_controller",
        executable="lights",
        name="lights_controller_node",  # Assign a unique name to the node
        output="screen",  # Show output in the console
        emulate_tty=True,  # Required for output to show in terminal when using 'screen'
        # parameters=[{'some_param': 'some_value'}] # Uncomment and add parameters if needed
    )

    # 2. Find the path to the 'talon.launch.py' file in the 'drive' package
    drive_package_share_dir = get_package_share_directory("drive")
    talon_launch_file_path = os.path.join(
        drive_package_share_dir, "launch", "talon.launch.py"
    )

    # Include the 'talon.launch.py' launch file
    talon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(talon_launch_file_path),
        # launch_arguments={'arg_name': 'arg_value'}.items() # Uncomment and add arguments if talon.launch.py expects them
    )

    # Return the LaunchDescription with both actions
    return LaunchDescription(
        [
            lights_node,
            talon_launch,
        ]
    )
