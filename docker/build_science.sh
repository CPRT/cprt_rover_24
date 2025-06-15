#!/bin/bash

cd "$(dirname "$0")"

AARCH=$(uname -m)
if [ "$AARCH" = "aarch64" ]; then
    echo "Detected ARM architecture."
    AARCH="arm64"
else
    echo "Detected AMD64 architecture."
    AARCH="amd64"
fi
echo "Using architecture: $AARCH"
cd ..

docker build -t cprtsoftware/cprt_rover_24-science:$AARCH \
             --build-arg AARCH="$AARCH" \
             -f docker/Dockerfile.science .

