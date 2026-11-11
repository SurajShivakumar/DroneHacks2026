#!/usr/bin/env bash
# Sensor-only OpenVINS check; never starts PX4 or external vision fusion.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
python3 -m sim.gazebo_smoke \
    --scene "$PWD/artifacts/gazebo-proxy-v8/world.sdf" \
    --hardware-rendering \
    --vio-config "$PWD/artifacts/vio-calibration-v1/estimator_config.yaml" \
    --output "$PWD/artifacts/gb10-vio-shadow-v1"
