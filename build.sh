#!/bin/bash

# Find all files tracked by git that are not part of a submodule
files=$(git ls-files -- ':!:(*/)' ':!:(*/**/*)' ':!:(*/**/*/*)')
for file in $files; do
  if [[ $file == *.cpp || $file == *.hpp || $file == *.c || $file == *.h ]]; then
    clang-format -i "$file"
  fi
  if [[ $file == *.py ]]; then
    black "$file"
  fi
done

rosdep install --from-paths src -i -r -y

colcon build --symlink-install --continue-on-error --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)