#!/usr/bin/env bash
set -euo pipefail
project_dir="$PWD"
test -f "$project_dir/AI_CONTEXT.md"
runtime_dir="$HOME/dronehacks-runtime"
python3 -m sim.gazebo_proxy \
    --models "$runtime_dir/PX4-Autopilot/Tools/simulation/gz/models" \
    --course "$project_dir/artifacts/course-v3-r6/course.sdf" \
    --output "$project_dir/artifacts/gazebo-proxy-v8"
