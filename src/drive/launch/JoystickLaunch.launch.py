import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def get_params(string, name, make_int: False):
    value = string.split(":")[1][1:]

    if make_int == False:
        return {name: value}
    else:
        return {name: int(value)}


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("drive"), "config")
    params_file = os.path.join(config_dir, "joystick_info.yaml")

    # Gets what joy node to launch and it's parameters
    with open(params_file, "r") as file:
        joy1_enabled = False
        joy2_enabled = False
        params1 = []
        params2 = []

        lines = file.readlines()
        for line in lines:
            line.strip()

            if joy1_enabled == False and line[:12] == "joy1_enabled":
                joy1_enabled = bool(line.split(":")[1][1:])
            elif joy2_enabled == False and line[:12] == "joy2_enabled":
                joy2_enabled = bool(line.split(":")[1][1:])

            if joy1_enabled == True:
                if line[:9] == "joy1_name":
                    params1.append(get_params(line, "joy1_name"))
                elif line[:7] == "joy1_id":
                    params1.append(get_params(line, "joy1_id", True))

            if joy2_enabled == True:
                if line[:9] == "joy2_name":
                    params2.append(get_params(line, "joy2_name"))
                elif line[:7] == "joy2_id":
                    params2.append(get_params(line, "joy2_id", True))

    # Creates joy nodes
    if joy1_enabled and joy2_enabled:
        joy1_topic = "/joy" + params1[1]["joy1_name"]
        joy2_topic = "/joy" + params2[1]["joy2_name"]

        if params1[1]["joy1_name"] == params2[1]["joy2_name"]:
            joy1_topic += "/" + params1[2]["joy1_id"]
            joy2_topic += "/" + params2[2]["joy2_id"]

            params1[1]["joy1_name"] = ""
            params2[1]["joy2_name"] = ""
        
        joy_launch_desc = [
            launch_ros.actions.Node(
                package="joy", 
                executable="joy_node", 
                name="joystick_1",
                remappings=[
                    ("/joy", joy1_topic)
                ],
                parameters=params1
            ),
            launch_ros.actions.Node(
                package="joy", 
                executable="joy_node", 
                name="joystick_2",
                remappings=[
                    ("/joy", joy2_topic)
                ],
                parameters=params2
            ),
        ]
    
    elif joy1_enabled:
        joy1_topic = "/joy" + params1[1]["joy1_name"]

        joy_launch_desc = [
            launch_ros.actions.Node(
                package="joy", 
                executable="joy_node", 
                name="joystick_1",
                remappings=[
                    ("/joy", joy1_topic)
                ],
                parameters=params1
            ),
        ]

    elif joy2_enabled:
        joy2_topic = "/joy" + params2[1]["joy2_name"]

        joy_launch_desc = [
            launch_ros.actions.Node(
                package="joy", 
                executable="joy_node", 
                name="joystick_2",
                remappings=[
                    ("/joy", joy2_topic)
                ],
                parameters=params2
            ),
        ]


    launch_descreiption = [
        launch_ros.actions.Node(
                package="drive",
                executable="joystick_drive",
                name="joystick_drive_station",
                parameters=[
                    {"PID_max_speed": 2.0},  # m/s
                    {"PID_max_turn": 2.0},  # rad/s
                    {"voltage_max_speed": 8.0},  # x/12volts
                    {"voltage_max_turn": 8.0},  # x/12volts
                    {"PID": 1},  # PID 1 Voltage 0
                ],
            ),
    ] + joy_launch_desc + [
        launch_ros.actions.Node(
                package="drive",
                executable="joystick_breakout",
                name="joystick_breakout_node",
                parameters=[params_file]
            ),
    ]

    return launch.LaunchDescription(launch_descreiption)
