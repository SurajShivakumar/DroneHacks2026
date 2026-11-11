#!/usr/bin/env bash
# Actual mission adapter and profile, DISARMED / READ-ONLY. No control publishers.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.gazebo_smoke --scene artifacts/sky-beta-estimated-v2/observer-fixture-world.sdf \
    --airframe-contract artifacts/sky-beta-estimated-v2/airframe-contract-4099.json \
    --px4-model-name sky_beta_estimated --hardware-rendering --record-image --record-topic /camera/observer/image \
    --with-px4 --external-vision --align-heading --diagnostic-rig --bootstrap-fixture \
    --ground-station "$HOME/dronehacks-runtime/qgc-v5.1.5/squashfs-root/AppRun" \
    --vio-config artifacts/sky-beta-vio-calibration-v1/estimator_config.yaml \
    --mission-settings configs/sky_beta_hover.json --output artifacts/gb10-sky-beta-mission-readonly-v2
