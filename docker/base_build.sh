#!/bin/bash

# Take in an arch argument to specify the architecture
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -a | --aarch) aarch="$2"; shift ;;
        -h| --help) 
            echo "Usage: $0 [-a <arm64|amd64|jetson>] [-h|--help]"
            echo "  -a, --aarch <arm64|amd64|jetson>  Specify the architecture (default is detected automatically)"
            echo "  -h, --help        Show this help message"
            exit 0 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Change directory to the script's location
cd "$(dirname "$0")"
source ./common.sh

# Check if Docker is installed
check_docker

tag=$(get_tag "$aarch")

image=ubuntu:22.04
if [ $tag="jetson" ]; then
    image=nvcr.io/nvidia/l4t-jetpack:r36.4.0
fi

# Build the Docker image
docker build -t cprtsoftware/cprt_rover_24-base:$tag \
             -f Dockerfile.base \
             --build-arg IMAGE=$image \
             ..
