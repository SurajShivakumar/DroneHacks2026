#!/usr/bin/env bash
# Simulated course run (user-approved repeat/speed series, Nov 6).
# Usage: gb10_sky_beta_course_run.sh <trajectory.csv> <settings.json> <output-dir>
set -eo pipefail
TRAJ="$1"; SETTINGS="$2"; OUT="$3"
[ -n "$OUT" ] || { echo "usage: $0 trajectory settings output" >&2; exit 2; }
source /opt/ros/jazzy/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
source "$HOME/dronehacks-runtime/vio_ws/install/setup.bash"
cd "$HOME/dronehacks-2026-gb10"
python3 -m sim.gazebo_smoke --scene artifacts/dronehacks_course_surface_arena_seed2026-v1/world.sdf \
  --airframe-contract artifacts/sky-beta-estimated-v4/airframe-contract.json \
  --px4-model-name sky_beta_estimated --record-image \
  --record-topic /camera/observer/image,/stereo/left/image,/stereo/right/image,/vio_shadow/trackhist \
  --record-vio-odometry --record-all-vio-features ${RECORD_BAG:+--record-bag} --estimator "${ESTIMATOR:-openvins}" ${CUVSLAM_INERTIAL:+--cuvslam-inertial $CUVSLAM_INERTIAL} ${NATIVE_PROGRAM:+--native-program "$NATIVE_PROGRAM"} ${SKYBETA_PROFILE:+--skybeta-profile} ${SKYBETA_MAG_TYPE:+--skybeta-mag-type $SKYBETA_MAG_TYPE} \
  --with-px4 --external-vision --align-heading --diagnostic-rig --bootstrap-fixture \
  --ground-station "$HOME/dronehacks-runtime/qgc-v5.1.5/squashfs-root/AppRun" \
  --vio-config "${VIO_CONFIG:-artifacts/vio-feature-budget-v1/estimator_config.yaml}" \
  --mission-settings "$SETTINGS" \
  --mission-repo teammate-review/dronehacks2026 \
  --mission-trajectory "$TRAJ" \
  --execute-mission \
  --approval-reference 'Suraj (Nov 6) asked to keep running simulated course flights for more successes and best times after v40; estimated Sky Beta dynamics, simulated takeoff/course/landing only. Native fresh preflight and all guards remain required.' \
  --output "$OUT"
