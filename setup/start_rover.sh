#!/bin/bash

# Setup
source ~/gstreamer/setupGstreamer.sh
source /opt/ros/humble/setup.bash
source /opt/ros/humble/cprt_setup.bash
sudo enablecan.sh

# Array of ros2 launch commands
launches=(
    "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    "ros2 run joy joy_node"
    "ros2 launch bringup arm_tasks.launch.py"
)

# Function to kill all child processes
cleanup() {
    echo "Shutting down all launches..."
    # Kill all background processes started by this script
    for pid in "${pids[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait
    exit 0
}

# Trap signals and call cleanup
trap cleanup SIGINT SIGTERM
sleep 10
# Start all launches in background and store PIDs
pids=()
for cmd in "${launches[@]}"; do
    echo "Starting: $cmd"
    $cmd &
    pids+=($!)
done

# Wait for all background processes
wait
