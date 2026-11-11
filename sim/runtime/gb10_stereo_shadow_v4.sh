#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
export ROS_DOMAIN_ID=42
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
python3 tools/observe_published_stereo.py --repo teammate-review/dronehacks2026 --calibration artifacts/stereo-depth-evidence-v2/calibration.yaml --airframe-contract artifacts/sky-beta-estimated-v4/airframe-contract.json --initialization configs/stereo_shadow_static_init.json --use-sim-time --fuse-shadow --seconds 180 --output artifacts/live-stereo-shadow-v4 > artifacts/live-stereo-shadow-v4.log 2>&1 &
observer_pid=$!
trap 'kill -INT "$observer_pid" 2>/dev/null || true; wait "$observer_pid" || true' EXIT
python3 -m sim.gazebo_smoke --scene artifacts/dronehacks_course_net_grid-v1/vio-forward-course-speed-v20-yaw.sdf --airframe-contract artifacts/sky-beta-estimated-v4/airframe-contract.json --px4-model-name sky_beta_estimated --record-image --record-topic /camera/observer/image,/stereo/left/image,/stereo/right/image,/vio_shadow/trackhist --record-vio-odometry --record-all-vio-features --diagnostic-rig-amplitude 0.8 --diagnostic-yaw-rad 0.35 --long-fixture-profile --with-px4 --external-vision --align-heading --diagnostic-rig --bootstrap-fixture --mission-settings configs/sky_beta_outbound_smooth.json --mission-observe-at-start --mission-observe-seconds 120 --ground-station "$HOME/dronehacks-runtime/qgc-v5.1.5/squashfs-root/AppRun" --vio-config artifacts/vio-feature-budget-v1/estimator_config.yaml --output artifacts/gb10-sky-beta-vio-quality-v24
