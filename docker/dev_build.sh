#!/bin/bash

# Take in an arch argument to specify the architecture
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -a | --aarch) aarch="$2"; shift ;;
        -h| --help) 
            echo "Usage: $0 [-a <arm64|amd64>] [-h|--help]"
            echo "  -a, --aarch <arm64|amd64>  Specify the architecture (default is detected automatically)"
            echo "  -h, --help        Show this help message"
            exit 0 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Change directory to the script's location
cd "$(dirname "$0")"

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if on a jetson device
is_jetson=0
if grep -q "NVIDIA Jetson" /proc/device-tree/model; then
    echo "Running on a Jetson device."
    is_jetson=1
else
    echo "Not running on a Jetson device."
fi
image=ubuntu:24.04
if [ $is_jetson -eq 1 ]; then
    docker build -t cprtsoftware/cprt_rover_24:jetson_base -f Dockerfile.jetson-base .
    image=cprtsoftware/cprt_rover_24:jetson_base
fi

# Check if arm or amd64 architecture
if [ -n "$aarch" ]; then
    if [ "$aarch" = "arm64" ]; then
        echo "Using ARM64 architecture as specified."
        aarch="arm64"
    elif [ "$aarch" = "amd64" ]; then
        echo "Using AMD64 architecture as specified."
        aarch="amd64"
    else
        echo "Invalid architecture specified. Use 'arm64' or 'amd64'."
        exit 1
    fi
else
    if [ "$(uname -m)" = "aarch64" ]; then
        echo "Detected ARM architecture."
        aarch="arm64"
    else
        echo "Detected AMD64 architecture."
        aarch="amd64"
    fi
fi

cd .

# Build the Docker image
docker build -t cprtsoftware/cprt_rover_24-dev:$aarch \
             -f Dockerfile.dev \
             --build-arg AARCH="$aarch" \
             --build-arg BASE_IMAGE=$image \
             ..
