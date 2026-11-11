#!/usr/bin/env bash
# Build upstream OpenVINS unmodified; does not run an estimator or flight process.
set -eo pipefail
test "$(id -u)" != 0
runtime_dir="$HOME/dronehacks-runtime"
mkdir -p "$runtime_dir/vio_ws/src"
cd "$runtime_dir/vio_ws"
if [ ! -d src/open_vins ]; then
    git clone --depth 1 https://github.com/rpng/open_vins.git src/open_vins
fi
git -C src/open_vins rev-parse HEAD > "$runtime_dir/openvins-source-commit.txt"
test -z "$(git -C src/open_vins status --porcelain --untracked-files=no)"
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon build --packages-select ov_core ov_init ov_msckf --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON
printf '\nOpenVINS built; no estimator started and no PX4 inputs published.\n'
