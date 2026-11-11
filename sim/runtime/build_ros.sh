#!/usr/bin/env bash
# Local user-prefix builds; no launch, flight commands or system installation.
set -eo pipefail
test "$(id -u)" != 0
runtime_dir="$HOME/dronehacks-runtime"
source /opt/ros/humble/setup.bash
cd "$runtime_dir/ros_ws"
colcon build --packages-select px4_msgs --executor sequential
cd "$runtime_dir/Micro-XRCE-DDS-Agent"
test "$(git describe --tags --exact-match)" = v2.4.3
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$runtime_dir/install" -DUAGENT_BUILD_TESTS=OFF
cmake --build build -j2
cmake --install build
printf '\nROS messages and XRCE agent built; neither agent nor autopilot started.\n'
