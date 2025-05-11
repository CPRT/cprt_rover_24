#!/bin/bash

IMAGE=cprtsoftware/cprt_rover_24
TAG=science-latest
JETSON_IP=192.168.0.55
PORT=9000

# Stop containers on exit
cleanup() {
    echo "Stopping containers..."
    docker stop servo_container sensor_container
    exit 0
}

trap cleanup SIGINT SIGTERM SIGHUP EXIT

# Shared args
DOCKER_ARGS="--network host --privileged -e RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} -v /dev/gpiomem:/dev/gpiomem"

# Run containers in detached mode
docker run -d --rm --name servo_container ${DOCKER_ARGS} ${IMAGE}:${TAG} \
    ros2 launch servo_pkg pi_servo_launch.launch.py

docker run -d --rm --name sensor_container ${DOCKER_ARGS} ${IMAGE}:${TAG} \
    ros2 launch science_sensors gas_sensor.launch.py

docker run -d --rm --name sensor_container ${DOCKER_ARGS} ${IMAGE}:${TAG} \
    ros2 launch science_sensors gpio.launch.py

gst-launch-1.0 libcamerasrc ! queue ! jpegenc ! jpegparse ! rtpjpegpay ! queue ! udpsink host=${JETSON_IP} port=${PORT} 

wait
