#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
python3 -m sim.gazebo_smoke \
    --scene "$PWD/artifacts/gazebo-proxy-v8/world.sdf" \
    --output "$PWD/artifacts/gazebo-sensor-smoke-v8"
