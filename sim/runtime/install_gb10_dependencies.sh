#!/usr/bin/env bash
# Admin step for the separately authorized GB10; never run in the managed lab.
# Installs packages only: no simulator, autopilot, GPU job or flight is started.
set -euo pipefail
source /etc/os-release
test "$ID" = ubuntu && test "$VERSION_ID" = 24.04
test "$(dpkg --print-architecture)" = arm64
test "$(id -u)" = 0
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends ca-certificates curl gnupg
key_file=/usr/share/keyrings/dronehacks-ros.gpg
source_file=/etc/apt/sources.list.d/dronehacks-ros2.list
# Preserve an existing ROS repository configured by the administrator.
if ! grep -RqsE 'packages.ros.org/ros2|repo.ros2.org' /etc/apt/sources.list.d /etc/apt/sources.list; then
    curl -fsSL --retry 3 https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o "$key_file"
    printf '%s\n' "deb [arch=arm64 signed-by=$key_file] http://packages.ros.org/ros2/ubuntu noble main" > "$source_file"
fi
apt-get update
# Jazzy brings Harmonic via ROS vendor packages; do not install Humble binaries.
apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base ros-jazzy-ros-gz \
    python3-colcon-common-extensions python3-venv python3-dev python3-pip \
    python3-pytest python3-numpy python3-pil python3-yaml ffmpeg \
    build-essential cmake ninja-build git pkg-config libssl-dev libxml2-dev \
    libeigen3-dev libopencv-dev libceres-dev libboost-all-dev \
    ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-tf2-geometry-msgs \
    gdb ccache rsync unzip zip
printf '\nGB10 dependencies installed. No simulator or flight process started.\n'
