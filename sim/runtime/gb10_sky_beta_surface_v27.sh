#!/usr/bin/env bash
# Disarmed appearance-sensitivity capture. No mission execution/arming.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
export ROS_DOMAIN_ID=42
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.extend_diagnostic_rail --source artifacts/dronehacks_course_surface_seed2026-v1/world.sdf --output artifacts/dronehacks_course_surface_seed2026-v1/vio-surface-v27.sdf --height-m 0.9 --travel-m 0.8 --velocity-mps 1.0
python3 -m sim.yaw_diagnostic artifacts/dronehacks_course_surface_seed2026-v1/vio-surface-v27.sdf artifacts/dronehacks_course_surface_seed2026-v1/vio-surface-v27-yaw.sdf --rate-rad-s 0.15
python3 -m sim.gazebo_smoke --scene artifacts/dronehacks_course_surface_seed2026-v1/vio-surface-v27-yaw.sdf --airframe-contract artifacts/sky-beta-estimated-v4/airframe-contract.json --px4-model-name sky_beta_estimated --record-image --record-topic /camera/observer/image,/stereo/left/image,/stereo/right/image,/vio_shadow/trackhist --record-vio-odometry --record-all-vio-features --diagnostic-rig-amplitude 0.8 --diagnostic-yaw-rad 1.57 --long-fixture-profile --with-px4 --external-vision --align-heading --diagnostic-rig --bootstrap-fixture --mission-settings configs/sky_beta_outbound_smooth.json --mission-observe-at-start --mission-observe-seconds 120 --ground-station "$HOME/dronehacks-runtime/qgc-v5.1.5/squashfs-root/AppRun" --vio-config artifacts/vio-feature-budget-v1/estimator_config.yaml --output artifacts/gb10-sky-beta-vio-surface-v27
