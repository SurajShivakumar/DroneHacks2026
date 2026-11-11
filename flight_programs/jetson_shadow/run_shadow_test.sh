#!/usr/bin/env bash
# run_shadow_test.sh - 60 s SHADOW test of upload_to_flyx_v4_skybeta.py on the Sky Beta Jetson.
#
# Run on the Jetson HOST (not in the container), drone on the bench, landed, DISARMED, props off:
#     bash run_shadow_test.sh
# Nothing here commands the vehicle: the program runs in shadow (publishes only /jolie_flyx/*),
# the checker only subscribes. It refuses to start if /skybeta/state says armed / offboard / not landed.
#
# Options:
#   --duration N         checker window, s (default 60, after a 5 s warm-up)
#   --no-fake-altitude   stay in TAKEOFF on the bench (default: fake altitude, so NAVIGATE + planner run)
#   --container NAME     default skybeta-stack
#   --dry-run            no docker / lsusb: run everything in this shell's ROS environment (tests, mocks)
#   --runs-dir DIR       where shadow-<time>/ goes (default ~/team_ws/runs; with --dry-run: ./runs)
#   --ready-timeout N    s to wait for the program to reach TAKEOFF (default 60)
# Exit: 0 PASS, 1 FAIL, 2 refused (preflight).
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
CONTAINER=skybeta-stack
DURATION=60
WARMUP=5
DRY=0
FAKE_ALT=1
READY_TIMEOUT=60
RUNS_DIR=""
CAMERA_OFFSET="0.1053 0 0.0304"        # Sky Beta TF base_link -> oak_parent_frame
DEPTH_TOPIC=/camera/depth/image_raw    # the team's depth_relay (lazy relay of /oak/stereo/*)
STAGE_CTR=/tmp/jetson_shadow
FILES="upload_to_flyx_v4_skybeta.py shadow_topic_check.py in_container.sh"

while [ $# -gt 0 ]; do
    case "$1" in
        --duration) DURATION="$2"; shift 2;;
        --no-fake-altitude) FAKE_ALT=0; shift;;
        --container) CONTAINER="$2"; shift 2;;
        --dry-run|--no-docker) DRY=1; shift;;
        --runs-dir) RUNS_DIR="$2"; shift 2;;
        --ready-timeout) READY_TIMEOUT="$2"; shift 2;;
        -h|--help) sed -n '2,20p' "$0"; exit 0;;
        *) echo "unknown option $1 (this script never flies)" >&2; exit 64;;
    esac
done

TS="$(date +%Y%m%d-%H%M%S)"
NAME="shadow-$TS"
if [ "$DRY" = 1 ]; then
    RUNS_HOST="${RUNS_DIR:-$PWD/runs}"; mkdir -p "$RUNS_HOST"; RUNS_HOST="$(cd "$RUNS_HOST" && pwd)"
    RUNS_CTR="$RUNS_HOST"
    STAGE="$RUNS_HOST/.stage-$TS"
else
    RUNS_HOST="${RUNS_DIR:-$HOME/team_ws/runs}"
    RUNS_CTR="/root/team_ws/runs"      # ~/team_ws is mounted at /root/team_ws (runs/ is root-owned)
    STAGE="$STAGE_CTR"
fi
HRUN="$RUNS_HOST/$NAME"                # as the host sees it
CRUN="$RUNS_CTR/$NAME"                 # as the container sees it
TMP="$(mktemp -d)"
DMESG_PID=""

say() { printf '%s\n' "$*"; }
ctr() {    # run a command string in the container's ROS environment
    if [ "$DRY" = 1 ]; then bash -c "$1"
    else docker exec "$CONTAINER" bash -ic "$1" 2> >(grep -v -e 'cannot set terminal process group' -e 'no job control' >&2)
    fi
}
ctr_write() {   # stdin -> file in the run dir
    if [ "$DRY" = 1 ]; then cat > "$1"; else docker exec -i "$CONTAINER" bash -c "cat > '$1'"; fi
}
cleanup() {
    [ -n "$DMESG_PID" ] && kill "$DMESG_PID" 2>/dev/null
    rm -rf "$TMP"
}
trap cleanup EXIT
refuse() { say ""; say "REFUSED: $*"; say "Nothing was started."; exit 2; }

say "=== Sky Beta shadow test $NAME  ($( [ "$DRY" = 1 ] && echo 'DRY RUN: no docker / lsusb' || echo "container $CONTAINER"))"

# ------------------------------------------------------------------ preflight
if [ "$DRY" = 0 ]; then
    [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = true ] \
        || refuse "container $CONTAINER is not running (docker ps)"
    say "container $CONTAINER: up"
    LSUSB="$(lsusb 2>/dev/null)"
    if printf '%s' "$LSUSB" | grep -qi '03e7:f63b'; then say "OAK camera: 03e7:f63b (booted)"
    elif printf '%s' "$LSUSB" | grep -qi '03e7:2485'; then refuse "OAK shows 03e7:2485 = UNBOOTED camera (USB dropped?). Restart the stack / replug, then retry"
    else refuse "no OAK camera (03e7:f63b) in lsusb"
    fi
    docker exec "$CONTAINER" mkdir -p "$STAGE" || refuse "cannot create $STAGE in the container"
    for f in $FILES; do docker cp "$HERE/$f" "$CONTAINER:$STAGE/$f" >/dev/null || refuse "docker cp $f failed"; done
else
    mkdir -p "$STAGE"; for f in $FILES; do cp "$HERE/$f" "$STAGE/$f"; done
fi
say "copied $FILES -> $STAGE"

PRE="$(ctr "bash $STAGE/in_container.sh preflight" 2>"$TMP/pre.err")"; PRE_RC=$?
say "vehicle: $PRE"
case "$PRE_RC" in
    0) ;;
    2) refuse "the vehicle reports ARMED / offboard / not landed. Disarm and land first; the shadow test never runs on an armed vehicle";;
    3) refuse "no /skybeta/state (is skybeta_flight_interface up? ROS_DOMAIN_ID?) $(tail -2 "$TMP/pre.err")";;
    4) refuse "a jolie_flyx node is already running: stop it first";;
    *) refuse "preflight failed (rc $PRE_RC): $(tail -5 "$TMP/pre.err")";;
esac
ctr "mkdir -p '$CRUN'" || refuse "cannot create $CRUN"
ctr "bash $STAGE/in_container.sh env '$CRUN'" | sed 's/^/  /'

# ------------------------------------------------------------------ USB monitor (host)
USB_NOTE=""
if sudo -n true 2>/dev/null; then
    sudo -n dmesg -w --time-format iso > "$TMP/dmesg.log" 2>&1 & DMESG_PID=$!; USB_NOTE="sudo -n dmesg -w"
elif dmesg >/dev/null 2>&1; then
    dmesg -w --time-format iso > "$TMP/dmesg.log" 2>&1 & DMESG_PID=$!; USB_NOTE="dmesg -w (no sudo needed)"
else
    USB_NOTE="SKIPPED: dmesg needs sudo and 'sudo -n' asks for a password (run 'sudo -v' first to include it)"
    : > "$TMP/dmesg.log"
fi
say "USB log: $USB_NOTE"
sleep 1

# ------------------------------------------------------------------ start the program (SHADOW)
PARGS="--depth-topic $DEPTH_TOPIC --camera-offset $CAMERA_OFFSET"
[ "$FAKE_ALT" = 1 ] && PARGS="$PARGS --fake-altitude"
say "starting upload_to_flyx_v4_skybeta.py --shadow $PARGS"
if [ "$DRY" = 1 ]; then
    bash "$STAGE/in_container.sh" start "$CRUN" $PARGS &
else
    docker exec -d "$CONTAINER" bash -ic "bash $STAGE/in_container.sh start '$CRUN' $PARGS"
fi
T_START=$(date +%s)
READY=""
while [ $(( $(date +%s) - T_START )) -lt "$READY_TIMEOUT" ]; do
    if [ -f "$HRUN/program.exit" ]; then break; fi
    if grep -qE -- '-> (TAKEOFF|NAVIGATE|SHADOW)' "$HRUN/program.log" 2>/dev/null; then READY=1; break; fi
    sleep 1
done
WAITED=$(( $(date +%s) - T_START ))
if [ -n "$READY" ]; then say "program ready after ${WAITED} s: $(grep -E -- '-> ' "$HRUN/program.log" | tail -1 | sed 's/.*\]: //')"
else say "program NOT ready after ${WAITED} s (checking anyway): $(tail -3 "$HRUN/program.log" 2>/dev/null)"
fi

# ------------------------------------------------------------------ checker
say "checker: ${WARMUP} s warm-up + ${DURATION} s ..."
ctr "bash $STAGE/in_container.sh check '$CRUN' --duration $DURATION --warmup $WARMUP --depth-topic $DEPTH_TOPIC" > "$TMP/check.out" 2>&1
CHECK_RC=$?

# ------------------------------------------------------------------ stop, collect
STOP_OUT="$(ctr "bash $STAGE/in_container.sh stop '$CRUN'" 2>&1)"
say "$STOP_OUT"
[ -n "$DMESG_PID" ] && { kill "$DMESG_PID" 2>/dev/null; wait "$DMESG_PID" 2>/dev/null; DMESG_PID=""; }
grep -iE 'usb|xhci|03e7|movidius|U1|U2' "$TMP/dmesg.log" > "$TMP/usb.log"
{ echo "# $USB_NOTE"; cat "$TMP/usb.log"; } | ctr_write "$CRUN/usb.log"
LSUSB_AFTER=""
if [ "$DRY" = 0 ]; then LSUSB_AFTER="$(lsusb 2>/dev/null | grep -i 03e7)"; fi
ctr "bash $STAGE/in_container.sh collect '$CRUN'" > /dev/null

# ------------------------------------------------------------------ verdict
FAILS=()
[ "$CHECK_RC" = 0 ] || FAILS+=("topic check: $(grep '^FAIL:' "$TMP/check.out" | sed 's/^FAIL: //' | paste -sd';' - | cut -c1-300)")
[ -n "$READY" ] || FAILS+=("program did not reach TAKEOFF/NAVIGATE within ${READY_TIMEOUT} s")
PEXIT="$(cat "$HRUN/program.exit" 2>/dev/null || echo '?')"
[ "$PEXIT" = 0 ] || FAILS+=("program exit code $PEXIT (expected 0 after SIGINT)")
grep -q 'jolie_flyx: clean shutdown' "$HRUN/program.log" 2>/dev/null || FAILS+=("no clean shutdown line in program.log")
[ -f "$HRUN/program.killed" ] && FAILS+=("program had to be SIGKILLed")
BAD_RX='\[(WARN|ERROR|FATAL)\]|Traceback|Error:|Warning:'
NBAD=$(grep -cE "$BAD_RX" "$HRUN/program.log" 2>/dev/null); NBAD=${NBAD:-0}
[ "$NBAD" = 0 ] || FAILS+=("$NBAD WARN/ERROR/Traceback line(s) in program.log")
NERR=$(grep -c '"kind": "error"' "$HRUN/journal.jsonl" 2>/dev/null); NERR=${NERR:-0}
[ "$NERR" = 0 ] || FAILS+=("$NERR error record(s) in journal.jsonl")
grep -q '"kind": "SHADOW_VIOLATION"' "$HRUN/journal.jsonl" 2>/dev/null && FAILS+=("SHADOW VIOLATION in journal")
NUSB=$(grep -cE 'USB disconnect|(U1|U2).*fail|fail.*(U1|U2)|reset (high|super|full)[- ]?speed' "$TMP/usb.log"); NUSB=${NUSB:-0}
[ "$NUSB" = 0 ] || FAILS+=("$NUSB USB disconnect/U1/U2/reset event(s) in dmesg")
if [ "$DRY" = 0 ] && ! printf '%s' "$LSUSB_AFTER" | grep -qi '03e7:f63b'; then
    FAILS+=("OAK not 03e7:f63b after the test: '${LSUSB_AFTER:-absent}'")
fi
VERDICT=PASS; [ ${#FAILS[@]} -eq 0 ] || VERDICT=FAIL

{
    say ""
    say "======================================================================================"
    say " SHADOW TEST $VERDICT    $NAME    (${DURATION} s window, fake altitude: $([ "$FAKE_ALT" = 1 ] && echo on || echo off))"
    say "======================================================================================"
    sed -n '/^key /,/^CHECK /p' "$TMP/check.out"
    say ""
    say "program log grep (WARN|ERROR|Traceback|stale|stall|foreign|timeout), first 15:"
    grep -nE 'WARN|ERROR|Traceback|stale|stall|foreign|timeout' "$HRUN/program.log" 2>/dev/null | cut -c1-200 | head -15 | sed 's/^/  /'
    say "  ($NBAD failing line(s); 'foreign publisher ... tolerated' INFO lines are expected while the square example runs)"
    say "program: exit $PEXIT; $(grep -E '^(RESULT|SHADOW):' "$HRUN/program.log" 2>/dev/null | cut -c1-220 | paste -sd'|' - | sed 's/|/\n         /g')"
    say "USB: $USB_NOTE; events: $NUSB$([ "$DRY" = 0 ] && echo "; lsusb after: ${LSUSB_AFTER:-absent}")"
    for f in "${FAILS[@]}"; do say "FAIL: $f"; done
    say "files: $HRUN/{program.log,checker_report.txt,checker.json,journal.jsonl,usb.log,env.txt,summary.txt}"
    say "RESULT: $VERDICT"
} > "$TMP/summary.txt"
cat "$TMP/summary.txt"
ctr_write "$CRUN/summary.txt" < "$TMP/summary.txt"
[ "$VERDICT" = PASS ]
