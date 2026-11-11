#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.extend_diagnostic_rail --source artifacts/dronehacks_course_net_grid-v1/world.sdf --output artifacts/dronehacks_course_net_grid-v1/vio-forward-course-speed-v13.sdf --travel-m 0.8 --velocity-mps 1.0
python3 -m sim.yaw_diagnostic artifacts/dronehacks_course_net_grid-v1/vio-forward-course-speed-v13.sdf artifacts/dronehacks_course_net_grid-v1/vio-forward-course-speed-v13-yaw.sdf --rate-rad-s 0.15
python3 -m sim.gazebo_smoke --scene artifacts/dronehacks_course_net_grid-v1/vio-forward-course-speed-v13-yaw.sdf --airframe-contract artifacts/sky-beta-estimated-v4/airframe-contract.json --px4-model-name sky_beta_estimated --record-image --record-topic /camera/observer/image,/stereo/left/image,/stereo/right/image,/vio_shadow/trackhist --record-vio-odometry --diagnostic-rig-amplitude 0.8 --diagnostic-yaw-rad 1.57 --with-px4 --external-vision --align-heading --diagnostic-rig --bootstrap-fixture --mission-settings configs/sky_beta_outbound_smooth.json --mission-observe-at-start --mission-observe-seconds 120 --ground-station "$HOME/dronehacks-runtime/qgc-v5.1.5/squashfs-root/AppRun" --vio-config artifacts/sky-beta-vio-calibration-v1/estimator_config.yaml --output artifacts/gb10-sky-beta-vio-quality-v13