#!/usr/bin/env bash
# Short constrained sensor-rig motion, not a flight or hardware test.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.sensor_rig artifacts/gazebo-proxy-v8/world.sdf artifacts/sensor-rig-v3/world.sdf
python3 -m sim.gazebo_smoke --scene artifacts/sensor-rig-v3/world.sdf \
    --hardware-rendering --diagnostic-rig --record-image \
    --vio-config artifacts/vio-calibration-v2/estimator_config.yaml \
    --output artifacts/gb10-vio-rig-v3
