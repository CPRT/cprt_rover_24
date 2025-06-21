#!/bin/bash

cd "$(dirname "$0")"

source ./docker/common.sh

aarch=$(uname -m)

colcon build --continue-on-error --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) \
             --build-base build-$aarch \
             --install-base install-$aarch

tar -czvf upgrade_pkg.tar.gz install-$aarch || echo "Tar failed" && exit 1
echo "Done"

