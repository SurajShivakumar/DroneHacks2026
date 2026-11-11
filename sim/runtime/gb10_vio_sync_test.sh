#!/usr/bin/env bash
# Compare shutdown after respecting synchronous processing; no flight.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
old="$HOME/dronehacks-runtime/vio_ws/build/watchdog-20261103T034600Z"
mkdir artifacts/vio-calibration-v2
cp "$old/estimator_config.yaml" "$old/kalibr_imucam_chain.yaml" "$old/kalibr_imu_chain.yaml" artifacts/vio-calibration-v2/
python3 -m sim.gazebo_smoke --scene artifacts/gazebo-proxy-v8/world.sdf \
  --hardware-rendering --vio-config artifacts/vio-calibration-v2/estimator_config.yaml \
  --output artifacts/gb10-vio-shadow-sync-v1
