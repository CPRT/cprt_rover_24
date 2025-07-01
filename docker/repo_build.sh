#!/bin/bash

commit_hash=$(git rev-parse --short HEAD)

echo "This script builds the Docker images for the cprt_rover_24 project."
echo "Requires private keys to access the cprtsoftware Docker repository."
echo "If you don't have access and feel like you need it, please ask the software lead."


# Change directory to the script's location
cd "$(dirname "$0")"
source ./common.sh


# Check if Docker is installed
check_docker

# Setup qemu
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# build base image
echo "Building local base image..."
docker buildx build \
    -t cprtsoftware/cprt_rover_24-base:latest \
    -f Dockerfile.base \
    --platform linux/amd64,linux/arm64/v8 \
    --build-arg IMAGE=stereolabs/zed:5.0-runtime-cuda12.8-ubuntu22.04 \
    --push \
    ..

echo "Building Jetson base image..."
docker buildx build \
    -t cprtsoftware/cprt_rover_24-jetson:latest \
    -f Dockerfile.jetson \
    --platform linux/arm64/v8 \
    --build-arg IMAGE=stereolabs/zed:5.0-runtime-l4t-r36.4 \
    --push \
    ..

echo "Building development image..."
docker buildx build \
    -t cprtsoftware/cprt_rover_24-dev:latest \
    -f Dockerfile.dev \
    --platform linux/amd64,linux/arm64/v8 \
    --build-arg IMAGE=cprtsoftware/cprt_rover_24-base:latest \
    --push \
    ..
