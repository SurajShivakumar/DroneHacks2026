#!/usr/bin/env bash
# Full estimated Sky Beta, actual VIO and PX4, always DISARMED.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.gazebo_smoke --scene artifacts/sky-beta-estimated-v2/observer-fixture-world.sdf \
    --airframe-contract artifacts/sky-beta-estimated-v2/airframe-contract.json \
    --px4-model-name sky_beta_estimated --hardware-rendering --record-image --record-topic /camera/observer/image \
    --with-px4 --external-vision --align-heading --diagnostic-rig --bootstrap-fixture \
    --ground-station "$HOME/dronehacks-runtime/qgc-v5.1.5/squashfs-root/AppRun" --inspect-offboard-disarmed \
    --vio-config artifacts/sky-beta-vio-calibration-v1/estimator_config.yaml \
    --output artifacts/gb10-sky-beta-preflight-v1
