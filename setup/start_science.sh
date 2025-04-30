#!/bin/bash
source /opt/ros/humble/setup.bash
source /opt/ros/humble/cprt_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cleanup() {
    echo "Killing child processes..."
    kill 0
    exit 0
}

trap cleanup SIGINT SIGTERM SIGHUP EXIT
