#!/bin/bash

check_docker() {
    if ! command -v docker &> /dev/null; then
        echo "Docker is not installed. Please install Docker first."
        exit 1
    fi
}

is_jetson() {
    if grep -q "NVIDIA Jetson" /proc/device-tree/model 2>/dev/null; then
        return 0
    else
        return 1
    fi
}

get_tag() {
    aarch=$1

    if [ -n "$aarch" ]; then
        # Proper use of OR (||) and spacing
        if [ "$aarch" != "arm64" ] && [ "$aarch" != "amd64" ] && [ "$aarch" != "jetson" ]; then
            echo "Invalid architecture specified. Use 'arm64', 'jetson', or 'amd64'."
            exit 1
        fi
        echo "$aarch"
    else
        # Call is_jetson and check the return value correctly
        if is_jetson; then
            echo "jetson"
        elif [ "$(uname -m)" = "aarch64" ]; then
            echo "arm64"
        else
            echo "amd64"
        fi
    fi
}
