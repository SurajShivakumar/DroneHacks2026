#!/usr/bin/env bash
# Native messages and user-prefix XRCE agent; does not launch transport or PX4.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
runtime_dir="$HOME/dronehacks-runtime"
mkdir -p "$runtime_dir/ros_ws/src"
if [ ! -d "$runtime_dir/ros_ws/src/px4_msgs" ]; then
    git clone --branch release/1.16 https://github.com/PX4/px4_msgs.git "$runtime_dir/ros_ws/src/px4_msgs"
    git -C "$runtime_dir/ros_ws/src/px4_msgs" checkout --detach 392e831c1f659429ca83902e66820d7094591410
fi
cd "$runtime_dir/ros_ws"
colcon build --packages-select px4_msgs --executor sequential
cd "$runtime_dir"
if [ ! -d Micro-XRCE-DDS-Agent ]; then
    git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
fi
test "$(git -C Micro-XRCE-DDS-Agent describe --tags --exact-match)" = v2.4.3
cmake -S Micro-XRCE-DDS-Agent -B Micro-XRCE-DDS-Agent/build -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$runtime_dir/install" -DUAGENT_BUILD_TESTS=OFF
cmake --build Micro-XRCE-DDS-Agent/build -j4
cmake --install Micro-XRCE-DDS-Agent/build
printf '\nNative ROS messages and XRCE agent built. No transport or flight started.\n'
