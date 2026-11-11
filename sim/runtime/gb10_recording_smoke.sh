#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
python3 -m sim.gazebo_smoke \
    --scene "$PWD/artifacts/gazebo-proxy-v8/world.sdf" \
    --hardware-rendering --record-image \
    --output "$PWD/artifacts/gb10-recording-smoke-v1"
python3 tools/encode_recording.py \
    "$PWD/artifacts/gb10-recording-smoke-v1/rgb-sequence" \
    "$PWD/artifacts/gb10-recording-smoke-v1/video"
