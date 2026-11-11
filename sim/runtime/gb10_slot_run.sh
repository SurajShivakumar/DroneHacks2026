#!/usr/bin/env bash
# Run one course flight in a private network + /tmp namespace so several flights
# can share GB10 without port/socket collisions (ROS domain, XRCE 8888, QGC 14550,
# /tmp/px4-sock-0). Requires kernel.apparmor_restrict_unprivileged_userns=0
# (set by the operator; not persisted). Flight code and guards are unchanged.
# Usage: gb10_slot_run.sh <trajectory.csv> <settings.json> <output-dir>
set -eo pipefail
cd "$HOME/dronehacks-2026-gb10"
[ -n "$3" ] || { echo "usage: $0 trajectory settings output" >&2; exit 2; }
LOG="$PWD/$3.launch.log"
U=$(id -u); G=$(id -g)
exec env VIO_CONFIG="${VIO_CONFIG:-}" RECORD_BAG="${RECORD_BAG:-}" ESTIMATOR="${ESTIMATOR:-openvins}" CUVSLAM_INERTIAL="${CUVSLAM_INERTIAL:-}" NATIVE_PROGRAM="${NATIVE_PROGRAM:-}" SKYBETA_PROFILE="${SKYBETA_PROFILE:-}" SKYBETA_MAG_TYPE="${SKYBETA_MAG_TYPE:-}" unshare -rnm sh -c "ip link set lo up && mount -t tmpfs none /tmp && exec unshare --map-user=$U --map-group=$G bash '$PWD/sim/runtime/gb10_sky_beta_course_run.sh' '$1' '$2' '$3'" > "$LOG" 2>&1
