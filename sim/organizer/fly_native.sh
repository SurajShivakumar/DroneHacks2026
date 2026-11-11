#!/usr/bin/env bash
# FlyX-lab style flight against a sim brought up by run_all.sh, recorded: a native PX4 program
# (team code publishes /fmu/in, as in the FlyX lab) instead of the skybeta flight_interface.
#   fly_native.sh <run-dir> <program.py> [program args...]
# e.g. fly_native.sh ~/runs/c1 flight_programs/flyx_course/fly_course.py
#      fly_native.sh ~/runs/p1 pathfinding/flyx/upload_to_flyx.py
# No flight_interface / competitor is started (two /fmu/in setpoint publishers would fight). The
# program arms and lands itself; this script only records, waits, and as a backstop lands with
# `commander land` if the program exits (or times out) while PX4 is still armed.
# Extra env: MISSION_TIMEOUT_S (default 420).
set -o pipefail
RUN="$(realpath -m "$1")"; PROG="$(realpath -m "$2")"; shift 2
HERE="$(cd "$(dirname "$0")" && pwd)"
[ -f "$HERE/env.sh" ] && source "$HERE/env.sh"
SIM="${ORGANIZER_SIM:-$HOME/organizer-sim}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-55}"          # same domain as run_px4.sh / run_stack.sh
SKYBETA_WS="${SKYBETA_WS:-$HOME/dronehacks-runtime/skybeta_ws}"
source /opt/ros/jazzy/setup.bash
source "$HERE/isaac_ros_env.sh"
source "$SKYBETA_WS/install/setup.bash"
# px4_msgs for the program (in SKYBETA_WS, or a separate underlay on some hosts)
python3 -c "import px4_msgs" 2>/dev/null || for ws in "$HOME/dronehacks-runtime/ros_ws" "${LAPTOP_PX4_MSGS_WS:-}"; do
  [ -f "$ws/install/setup.bash" ] && source "$ws/install/setup.bash" && break; done
python3 -c "import px4_msgs" || { echo "px4_msgs not found"; exit 1; }
PX4BIN="${PX4_DIR:-$SIM/PX4-Autopilot-v1.16.2}/build/px4_sitl_default/bin"
OUT="$RUN/flight-native-$(basename "$PROG" .py)-$(date +%H%M%S)"; mkdir -p "$OUT"
log() { echo "[fly $(date +%T)] $*" | tee -a "$OUT/fly.log"; }
start() { setsid "${@:2}" > "$OUT/$1.log" 2>&1 < /dev/null & echo "$1 $!" >> "$RUN/pids"; echo $!; }
arming() { (cd "$RUN/px4" && timeout 5 "$PX4BIN/px4-listener" --instance 0 vehicle_status 2>/dev/null) \
             | grep -m1 -oE "arming_state: [0-9]+" | grep -oE "[0-9]+$"; }

start recorder python3 "$HERE/record_flight.py" --output "$OUT" > /dev/null
log "program: $PROG $*"
PID=$(cd "$(dirname "$PROG")" && start program python3 -u "$PROG" "$@")
T0=$(date +%s)
while kill -0 "$PID" 2>/dev/null; do
  if [ $(( $(date +%s) - T0 )) -gt "${MISSION_TIMEOUT_S:-420}" ]; then log "timeout: stopping the program"; kill -INT -- "-$PID"; sleep 5; break; fi
  sleep 2
done
log "program exited: $(tail -1 "$OUT/program.log")"
if [ "$(arming)" = 2 ]; then
  log "still armed: pilot lands"; (cd "$RUN/px4" && "$PX4BIN/px4-commander" --instance 0 land) 2>&1 | tail -2
  for _ in $(seq 150); do [ "$(arming)" = 2 ] || break; sleep 2; done
fi
log "disarmed (arming_state $(arming))"
sleep 3; log "done"
