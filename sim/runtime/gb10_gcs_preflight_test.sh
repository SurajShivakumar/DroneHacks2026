#!/usr/bin/env bash
# Real GCS + released DISARMED aircraft + read-only hover preflight inspection.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.gazebo_smoke --scene artifacts/bootstrap-fixture-v1/world.sdf \
    --hardware-rendering --diagnostic-rig --bootstrap-fixture --with-px4 --external-vision --record-image \
    --ground-station "$HOME/dronehacks-runtime/qgc-v5.1.5/squashfs-root/AppRun" --inspect-hover --align-heading \
    --vio-config artifacts/course-rig-calibration-v1/estimator_config.yaml \
    --output artifacts/gb10-gcs-preflight-v2
