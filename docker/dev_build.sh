#!/bin/bash

# Change directory to the script's location
cd "$(dirname "$0")"

source ./common.sh

# Check if Docker is installed
check_docker()

tag=get_tag
# Check if on a jetson device
image=cprtsoftware/cprt_rover_24-base:$tag

# Build the Docker image
docker build -t cprtsoftware/cprt_rover_24-dev:$tag \
             -f Dockerfile.dev \
             --build-arg BASE_IMAGE=$image \
             ..
