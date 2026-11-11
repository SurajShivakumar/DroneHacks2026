#!/usr/bin/env bash
# Bounded, sensor-only test. No PX4, agent, setpoints or arming.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
python3 -m sim.gazebo_smoke \
    --scene "$PWD/artifacts/gazebo-proxy-v8/world.sdf" \
    --hardware-rendering \
    --output "$PWD/artifacts/gb10-sensor-smoke-v1"
