#!/usr/bin/env bash
# Bounded simulated corner qualification; intentionally not a full-course run.
# Estimated Sky Beta; no hardware flight is authorized by this script.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.gazebo_smoke --scene artifacts/dronehacks_course_net_grid-v1/world.sdf \
    --airframe-contract artifacts/sky-beta-estimated-v4/airframe-contract.json \
    --px4-model-name sky_beta_estimated --record-image \
    --record-topic /camera/observer/image,/stereo/left/image,/stereo/right/image,/vio_shadow/trackhist \
    --record-vio-odometry \
    --with-px4 --external-vision --align-heading --diagnostic-rig --bootstrap-fixture \
    --ground-station "$HOME/dronehacks-runtime/qgc-v5.1.5/squashfs-root/AppRun" \
    --vio-config artifacts/vio-feature-budget-v1/estimator_config.yaml \
    --mission-settings configs/sky_beta_corner_qualification.json --mission-repo teammate-review/dronehacks2026 --mission-trajectory artifacts/sky-beta-corner-reference-v1/trajectory.csv --execute-mission \
    --approval-reference 'Suraj requested a simulated launch/fly/land run and authorized estimated Sky Beta dynamics in this thread' \
    --output artifacts/gb10-sky-beta-corner-qualification-v1
