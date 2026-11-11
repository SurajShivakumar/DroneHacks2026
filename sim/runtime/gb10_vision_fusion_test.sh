#!/usr/bin/env bash
# Real VIO-to-PX4 fusion under DISARMED guard, no flight/control commands.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.world_environment artifacts/course-rig-v1/world.sdf \
  "$HOME/dronehacks-runtime/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf" \
  artifacts/course-rig-environment-v1/world.sdf
python3 -m sim.gazebo_smoke --scene artifacts/course-rig-environment-v1/world.sdf \
  --hardware-rendering --with-px4 --external-vision --diagnostic-rig --record-image \
  --px4-model-name diagnostic_sensor_rig \
  --vio-config artifacts/course-rig-calibration-v1/estimator_config.yaml \
  --output artifacts/gb10-vision-fusion-v1
