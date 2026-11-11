#!/usr/bin/env bash
# Diagnose estimator startup without sensors, PX4 or flight processes.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
timeout 25 gdb --batch -ex run -ex 'bt 18' --args \
  "$HOME/dronehacks-runtime/vio_ws/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf" \
  "$PWD/artifacts/vio-calibration-v1/estimator_config.yaml" --ros-args -p use_sim_time:=true
