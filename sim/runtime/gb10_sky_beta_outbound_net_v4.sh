#!/usr/bin/env bash
# Sim-only first qualification within the user's requested launch/fly/land task.
# Estimated Sky Beta; no hardware flight is authorized by this script.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.gazebo_smoke --scene artifacts/dronehacks_course_net_grid-v1/world.sdf \
    --airframe-contract artifacts/sky-beta-estimated-v4/airframe-contract.json \
    --px4-model-name sky_beta_estimated --hardware-rendering --record-image --record-topic /camera/observer/image \
    --with-px4 --external-vision --align-heading --diagnostic-rig --bootstrap-fixture \
    --ground-station "$HOME/dronehacks-runtime/qgc-v5.1.5/squashfs-root/AppRun" \
    --vio-config artifacts/sky-beta-vio-calibration-v1/estimator_config.yaml \
    --mission-settings configs/sky_beta_outbound_path_sync.json --mission-repo teammate-review/dronehacks2026 --mission-trajectory artifacts/teammate-reproduction-v2/nominal-000.csv --execute-mission \
    --approval-reference 'Suraj requested a simulated launch/fly/land run and authorized estimated Sky Beta dynamics in this thread' \
    --output artifacts/gb10-sky-beta-outbound-net-v4
