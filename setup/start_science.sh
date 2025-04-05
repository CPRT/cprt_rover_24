#!/bin/bash

source /opt/ros/humble/setup.bash
source /opt/ros/humble/cprt_setup.bash
source ~/.bashrc
ros2 launch rosbridge_server rosbridge_websocket_launch.xml