#!/usr/bin/env bash
# Sim-only diagnostic retry; preserve all previous evidence under the v1 name.
# Estimated Sky Beta; no hardware flight is authorized by this script.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.gazebo_smoke --scene artifacts/dronehacks_course_net_grid-v1/world.sdf \
    --airframe-contract artifacts/sky-beta-estimated-v4/airframe-contract.json \
    --px4-model-name sky_beta_estimated --hardware-rendering --record-image \
    --record-topic /camera/observer/image,/stereo/left/image,/stereo/right/image,/vio_shadow/trackhist \
    --record-vio-odometry \
    --with-px4 --external-vision --align-heading --diagnostic-rig --bootstrap-fixture \
    --ground-station "$HOME/dronehacks-runtime/qgc-v5.1.5/squashfs-root/AppRun" \
    --vio-config artifacts/sky-beta-vio-calibration-v1/estimator_config.yaml \
    --mission-settings configs/sky_beta_astar.json --mission-repo teammate-review/dronehacks2026 --mission-trajectory artifacts/published-course-astar-v1/trajectory.csv --execute-mission \
    --approval-reference 'Suraj requested a simulated launch/fly/land run and authorized estimated Sky Beta dynamics in this thread' \
    --output artifacts/gb10-sky-beta-astar-net-v2
