#!/usr/bin/env bash
# Inspection experiment only; this hidden-airframe scene is not a flight baseline.
set -eo pipefail
source /opt/ros/humble/setup.bash
python3 -m sim.gazebo_proxy \
    --models "$HOME/dronehacks-runtime/PX4-Autopilot/Tools/simulation/gz/models" \
    --course "$PWD/artifacts/course-v3-r4/course.sdf" \
    --output "$PWD/artifacts/occlusion-diagnostic-world" --diagnostic-hide-airframe-visuals
python3 -m sim.gazebo_smoke --scene "$PWD/artifacts/occlusion-diagnostic-world/world.sdf" \
    --output "$PWD/artifacts/occlusion-diagnostic-v1"
