#!/usr/bin/env bash
# Bounded non-flight check. No arming, takeoff, offboard or setpoint commands.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
python3 -m sim.gazebo_smoke \
    --scene "$PWD/artifacts/gazebo-proxy-v5/world.sdf" \
    --output "$PWD/artifacts/gazebo-px4-smoke-v5" --with-px4
