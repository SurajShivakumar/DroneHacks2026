#!/usr/bin/env bash
# Course-only visual scene; constrained sensor rig, no aircraft flight.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.sensor_rig artifacts/gazebo-proxy-v8/world.sdf artifacts/course-rig-v1/world.sdf \
    --no-checkerboard --height .3 --camera-pitch .35 --down-rgb
python3 -m sim.vio_calibration --world artifacts/course-rig-v1/world.sdf \
    --observations artifacts/gb10-vio-rig-v4/ros-observation.json \
    --model-name diagnostic_sensor_rig --output artifacts/course-rig-calibration-v1
# Preserve the already parsed complete estimator options; use newly derived TF.
cp artifacts/vio-calibration-v2/estimator_config.yaml artifacts/course-rig-calibration-v1/estimator_config.yaml
python3 -m sim.gazebo_smoke --scene artifacts/course-rig-v1/world.sdf \
    --hardware-rendering --diagnostic-rig --record-image \
    --vio-config artifacts/course-rig-calibration-v1/estimator_config.yaml \
    --output artifacts/gb10-course-rig-v1
