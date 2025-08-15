#!/bin/bash

REPO=cprtsoftware/cprt_rover_24
TYPE=jetson # Replace with 'base' as needed
TAG=latest # Replace with the desired commit hash or 'latest'

BASE_IMAGE_NAME="$REPO-$TYPE:$TAG"
IMAGE_NAME="$REPO-$TYPE-deploy:$TAG"

cd "$(dirname "$0")"

source ./docker/common.sh

files=$(git ls-files -- ':!:(*/)' ':!:(*/**/*)' ':!:(*/**/*/*)')

# Filter out python files into one variable
python_files=$(
  for file in $files; do
    if [[ $file == *.py ]]; then
      echo "$file"
    fi
  done
)

# Launch one instance of `black` for all python files
IFS=$'\n' black $python_files

# Format C/C++ source & header files
for file in $files; do
  if [[ $file == *.cpp || $file == *.hpp || $file == *.c || $file == *.h ]]; then
    clang-format -i "$file"
  fi
done

check_docker

docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

docker buildx build \
    -t $IMAGE_NAME \
    -f Dockerfile \
    --platform linux/arm64/v8 \
    --build-arg IMAGE=stereolabs/zed:5.0-runtime-l4t-r36.4 \
    --push \
    .

