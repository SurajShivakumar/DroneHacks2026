#!/usr/bin/env bash
# Observe real stereo/IMU through unmodified OpenVINS. No PX4 is started.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
python3 -m sim.gazebo_smoke \
    --scene "$PWD/artifacts/gazebo-proxy-v8/world.sdf" \
    --vio-config "$PWD/artifacts/vio-calibration-v1/estimator_config.yaml" \
    --output "$PWD/artifacts/vio-shadow-v2"
