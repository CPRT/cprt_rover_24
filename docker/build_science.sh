#!/bin/bash

# Change directory to the script's location
cd "$(dirname "$0")"

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

source ./common.sh

# Check if Docker is installed
check_docker()

tag=$(get_tag "$aarch")
image=cprtsoftware/cprt_rover_24-base:$tag

# Build the Docker image
docker build -t cprtsoftware/cprt_rover_24-science:$tag \
             -f Dockerfile.dev \
             --build-arg BASE_IMAGE=$image \
             ..
