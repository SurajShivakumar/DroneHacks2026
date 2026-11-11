#!/usr/bin/env bash
# Local WSL Ubuntu 22.04 only. Never run this in the managed companion lab.
set -euo pipefail
source /etc/os-release
test "$ID" = ubuntu && test "$VERSION_ID" = 22.04
test "$(id -u)" = 0
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y --no-install-recommends ca-certificates curl gnupg lsb-release
curl -fsSL --retry 3 https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/dronehacks-osrf.gpg
curl -fsSL --retry 3 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/dronehacks-ros.gpg
printf '%s\n' 'deb [arch=amd64 signed-by=/usr/share/keyrings/dronehacks-osrf.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main' > /etc/apt/sources.list.d/dronehacks-gazebo.list
printf '%s\n' 'deb [arch=amd64 signed-by=/usr/share/keyrings/dronehacks-ros.gpg] http://packages.ros.org/ros2/ubuntu jammy main' > /etc/apt/sources.list.d/dronehacks-ros2.list
apt-get update -qq
apt-get install -y --no-install-recommends \
    gz-harmonic ros-humble-ros-base ros-humble-ros-gzharmonic \
    python3-colcon-common-extensions python3-venv python3-dev python3-pip \
    build-essential cmake ninja-build git pkg-config libssl-dev libxml2-dev \
    libxml2-utils libeigen3-dev libopencv-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    astyle ccache cppcheck file gdb lcov rsync shellcheck unzip zip
printf '\nInstalled local proxy dependencies; no simulator/autopilot has been started.\n'
