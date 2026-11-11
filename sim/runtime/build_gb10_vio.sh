#!/usr/bin/env bash
# Native ARM64/Jazzy build of pinned, unmodified OpenVINS. No flight processes.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
runtime_dir="$HOME/dronehacks-runtime"
mkdir -p "$runtime_dir/vio_ws/src"
cd "$runtime_dir/vio_ws"
if [ ! -d src/open_vins ]; then
    git clone https://github.com/rpng/open_vins.git src/open_vins
    git -C src/open_vins checkout --detach 69488123ed9362dd44b6f28e7f4680abbff1442b
fi
test "$(git -C src/open_vins rev-parse HEAD)" = 69488123ed9362dd44b6f28e7f4680abbff1442b
test -z "$(git -C src/open_vins status --porcelain --untracked-files=no)"
git -C src/open_vins rev-parse HEAD > "$runtime_dir/openvins-source-commit.txt"
export CMAKE_BUILD_PARALLEL_LEVEL=4
export MAKEFLAGS=-j4
colcon build --packages-select ov_core ov_init ov_msckf --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON
printf '\nNative OpenVINS build finished; no estimator or flight started.\n'
