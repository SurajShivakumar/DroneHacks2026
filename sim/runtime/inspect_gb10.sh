#!/usr/bin/env bash
# Read-only post-install check; no GPU job or simulator is started.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
uname -m
python3 --version
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix gz_sim_vendor
gz sim --versions
dpkg-query -W 'ros-jazzy-ros-gz*' 'ros-jazzy-gz-*-vendor'
printf '\nReady for a bounded sensor check, not qualified for flight.\n'
