#!/usr/bin/env bash
# User-approved simulated takeoff/course/landing; estimated exterior scenario.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.gazebo_smoke --scene artifacts/stereo-arena-1280x800-v2/world.sdf \
  --airframe-contract artifacts/sky-beta-estimated-v4/airframe-contract.json \
  --px4-model-name sky_beta_estimated --record-image \
  --record-topic /camera/observer/image,/stereo/left/image,/stereo/right/image,/vio_shadow/trackhist \
  --record-vio-odometry --record-all-vio-features \
  --with-px4 --external-vision --align-heading --diagnostic-rig --bootstrap-fixture \
  --ground-station "$HOME/dronehacks-runtime/qgc-v5.1.5/squashfs-root/AppRun" \
  --vio-config artifacts/vio-capacity-1200-v1/estimator_config.yaml \
  --mission-settings configs/sky_beta_first_completion.json \
  --mission-repo teammate-review/dronehacks2026 \
  --mission-trajectory artifacts/published-course-efficient-v1/trajectory.csv \
  --execute-mission \
  --approval-reference 'Suraj requested one proper full simulated takeoff/checkpoints/landing run despite speed; this thread authorizes estimated Sky Beta dynamics and simulated course flight. Native fresh preflight and all guards remain required.' \
  --output artifacts/gb10-sky-beta-first-completion-v32
