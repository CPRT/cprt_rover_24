#!/bin/bash

# Change directory to the script's location
cd "$(dirname "$0")"

source ./common.sh

# Check if Docker is installed
check_docker

tag=$(get_tag)
image=cprtsoftware/cprt_rover_24-base:$tag

# Build the Docker image
docker build -t cprtsoftware/cprt_rover_24:$tag \
             -f Dockerfile.deploy \
             --build-arg IMAGE=$image \
             ..
