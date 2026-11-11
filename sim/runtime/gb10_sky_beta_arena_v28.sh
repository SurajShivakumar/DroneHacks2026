#!/usr/bin/env bash
# Disarmed native-estimator check at the planned slow translation speed.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
export ROS_DOMAIN_ID=42
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.extend_diagnostic_rail --source artifacts/dronehacks_course_arena_seed2026-v1/world.sdf --output artifacts/dronehacks_course_arena_seed2026-v1/vio-arena-v28.sdf --height-m 0.9 --travel-m 0.8 --velocity-mps 0.25
python3 -m sim.gazebo_smoke --scene artifacts/dronehacks_course_arena_seed2026-v1/vio-arena-v28.sdf --airframe-contract artifacts/sky-beta-estimated-v4/airframe-contract.json --px4-model-name sky_beta_estimated --record-image --record-topic /camera/observer/image,/stereo/left/image,/stereo/right/image,/vio_shadow/trackhist --record-vio-odometry --record-all-vio-features --diagnostic-rig-amplitude 0.8 --long-fixture-profile --with-px4 --external-vision --align-heading --diagnostic-rig --bootstrap-fixture --mission-settings configs/sky_beta_outbound_smooth.json --mission-observe-at-start --mission-observe-seconds 120 --ground-station "$HOME/dronehacks-runtime/qgc-v5.1.5/squashfs-root/AppRun" --vio-config artifacts/vio-feature-budget-v1/estimator_config.yaml --output artifacts/gb10-sky-beta-vio-arena-v28
