#!/bin/bash

set -euo pipefail

commit_hash=$(git rev-parse --short HEAD)

local=false
jetson=false
dev=false

# Take in an arch argument to specify the architecture
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -a| --all) 
            local=true
            jetson=true
            dev=true ;;
        -j| --jetson) 
            jetson=true ;;
        -d| --dev) 
            dev=true ;;
        --local) 
            local=true ;;
        -h| --help) 
            echo "Usage: $0 [--local] [--jetson] [--dev] [-a|--all] [-h|--help]"
            echo "Options:"
            echo "  -a, --all        Build all types of images"
            echo "  -j, --jetson     Build base for Jetson"
            echo "  -d, --dev        Build development image"
            echo "  --local          Build local base image"
            echo "  -h, --help       Show this help message"
            exit 0 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

if [[ "$local" == false && "$jetson" == false && "$dev" == false ]]; then
    echo "No build type specified. Use -a, -j, -d, or --local to specify a build type."
    exit 1
fi

# Change directory to the script's location
cd "$(dirname "$0")"
source ./common.sh


# Check if Docker is installed
check_docker

# Setup qemu
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# build base image
if $local ; then
    echo "Building local base image..."
    docker buildx build \
        -t cprtsoftware/cprt_rover_24-base:$commit_hash \
        -f Dockerfile.base \
        --build-arg IMAGE=stereolabs/zed:5.0-devel-cuda12.8-ubuntu22.04 \
        --load \
        ..
fi

if $jetson ; then
    echo "Building Jetson base image..."
    docker buildx build\
        -t cprtsoftware/cprt_rover_24-jetson:$commit_hash \
        -f Dockerfile.base \
        --platform linux/arm64/v8 \
        --build-arg IMAGE=stereolabs/zed:5.0-tools-devel-l4t-r36.4 \
        --load \
        ..
fi

if $dev ; then
    echo "Building development image..."
    docker buildx build\
        -t cprtsoftware/cprt_rover_24-dev:$commit_hash \
        -f Dockerfile.dev \
        --build-arg IMAGE=cprtsoftware/cprt_rover_24-base:$commit_hash \
        --load \
        ..
fi