#!/usr/bin/env bash
# Preflight physical motion + release, remains DISARMED throughout.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.bootstrap_fixture artifacts/gazebo-proxy-v8/world.sdf artifacts/bootstrap-fixture-v1/source.sdf
python3 -m sim.world_environment artifacts/bootstrap-fixture-v1/source.sdf \
    "$HOME/dronehacks-runtime/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf" \
    artifacts/bootstrap-fixture-v1/world.sdf
python3 -m sim.gazebo_smoke --scene artifacts/bootstrap-fixture-v1/world.sdf \
    --hardware-rendering --diagnostic-rig --bootstrap-fixture --with-px4 --external-vision --record-image \
    --vio-config artifacts/course-rig-calibration-v1/estimator_config.yaml \
    --output artifacts/gb10-bootstrap-v1
