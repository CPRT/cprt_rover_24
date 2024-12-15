#!/bin/bash

rosdep install --from-paths src -i -r -y

colcon build --symlink-install --continue-on-error --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)