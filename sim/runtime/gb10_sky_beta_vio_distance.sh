#!/usr/bin/env bash
# Disarmed repeated-distance VIO check; no arm or Offboard commands.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.extend_diagnostic_rail \
    --source artifacts/dronehacks_course_net_grid-v1/world.sdf \
    --output artifacts/dronehacks_course_net_grid-v1/vio-distance-world.sdf \
    --travel-m 0.8
python3 -m sim.gazebo_smoke --scene artifacts/dronehacks_course_net_grid-v1/vio-distance-world.sdf \
    --airframe-contract artifacts/sky-beta-estimated-v4/airframe-contract.json \
    --px4-model-name sky_beta_estimated --hardware-rendering --record-image \
    --record-topic /camera/observer/image,/stereo/left/image,/stereo/right/image,/vio_shadow/trackhist \
    --record-vio-odometry --diagnostic-rig-amplitude 0.8 \
    --with-px4 --external-vision --align-heading --diagnostic-rig --bootstrap-fixture \
    --ground-station "$HOME/dronehacks-runtime/qgc-v5.1.5/squashfs-root/AppRun" \
    --vio-config artifacts/sky-beta-vio-calibration-v1/estimator_config.yaml \
    --output artifacts/gb10-sky-beta-vio-quality-v3
