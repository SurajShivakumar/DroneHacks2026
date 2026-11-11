#!/usr/bin/env bash
# Container-side helper for run_shadow_test.sh (runs INSIDE skybeta-stack, or locally with --dry-run).
# Never publishes anything itself; "start" launches the program in SHADOW (it has no --fly here).
#
#   in_container.sh preflight                 -> shadow_topic_check.py --preflight (exit 0 ok, 2 armed, 3 no state, 4 running)
#   in_container.sh env   RUN
#   in_container.sh start RUN [program args]  -> RUN/program.log, RUN/program.pid, RUN/program.exit
#   in_container.sh check RUN [checker args]  -> RUN/checker_report.txt, RUN/checker.json
#   in_container.sh stop  RUN                 -> SIGINT the program (by PID), wait, then TERM / KILL
#   in_container.sh collect RUN               -> RUN/journal.jsonl
#   in_container.sh write FILE                -> stdin to FILE (the host cannot write the root-owned runs dir)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
if [ -z "${ROS_DISTRO:-}" ]; then
    set +u; for f in /opt/ros/humble/setup.bash /opt/ros/*/setup.bash; do [ -f "$f" ] && . "$f" && break; done; set -u
fi
export RCUTILS_COLORIZED_OUTPUT=0 PYTHONUNBUFFERED=1
cmd="${1:-}"; shift || true

case "$cmd" in
preflight)
    exec python3 "$HERE/shadow_topic_check.py" --preflight "$@"
    ;;
env)
    RUN="$1"; mkdir -p "$RUN"
    {
        echo "date: $(date -Is)"; echo "host: $(uname -a)"
        echo "ROS_DISTRO=${ROS_DISTRO:-} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-}"
        echo "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
        python3 - <<'EOF'
import sys
print("python", sys.version.split()[0])
import numpy; print("numpy", numpy.__version__)
try:
    import scipy; print("scipy", scipy.__version__, "(used)")
except ImportError:
    print("scipy: not installed (numpy fallback)")
try:
    import px4_msgs.msg; print("px4_msgs: importable")
except ImportError:
    print("px4_msgs: NOT importable (sensor_combined check skipped)")
EOF
        sha256sum "$HERE"/upload_to_flyx_v4_skybeta.py "$HERE"/shadow_topic_check.py
    } > "$RUN/env.txt" 2>&1
    cat "$RUN/env.txt"
    ;;
start)
    RUN="$1"; shift; mkdir -p "$RUN"
    for arg in "$@"; do
        case "$arg" in --fly|--execute|--fl|--f) echo "REFUSING: the shadow test never flies ($arg)" > "$RUN/program.log"; echo 64 > "$RUN/program.exit"; exit 64;; esac
    done
    rm -f "$RUN/program.pid" "$RUN/program.exit"
    cd "$RUN" || exit 1
    # the inner bash writes its own PID, then exec's python3 (same PID): we signal exactly that process
    bash -c 'echo $$ > "$0/program.pid"; exec python3 -u "$@"' "$RUN" \
        "$HERE/upload_to_flyx_v4_skybeta.py" --shadow --output "$RUN/program" "$@" > "$RUN/program.log" 2>&1
    echo $? > "$RUN/program.exit"
    ;;
check)
    RUN="$1"; shift
    python3 "$HERE/shadow_topic_check.py" --json "$RUN/checker.json" "$@" 2>&1 | tee "$RUN/checker_report.txt"
    exit "${PIPESTATUS[0]}"
    ;;
stop)
    RUN="$1"
    [ -f "$RUN/program.pid" ] || { echo "no program.pid"; exit 1; }
    PID="$(cat "$RUN/program.pid")"
    if ! kill -0 "$PID" 2>/dev/null; then echo "program $PID already exited"; exit 0; fi
    echo "SIGINT -> program PID $PID"
    kill -INT "$PID"
    for _ in $(seq 1 200); do kill -0 "$PID" 2>/dev/null || break; sleep .1; done
    if kill -0 "$PID" 2>/dev/null; then
        echo "still running after 20 s: SIGTERM"; kill -TERM "$PID"; sleep 5
    fi
    if kill -0 "$PID" 2>/dev/null; then
        echo "still running: SIGKILL (and its planner child)"
        pkill -KILL -P "$PID" 2>/dev/null; kill -KILL "$PID"
        echo "killed" > "$RUN/program.killed"
    fi
    for _ in $(seq 1 50); do [ -f "$RUN/program.exit" ] && break; sleep .1; done
    echo "program exit code: $(cat "$RUN/program.exit" 2>/dev/null || echo '?')"
    ;;
collect)
    RUN="$1"
    [ -f "$RUN/program/journal.jsonl" ] && cp "$RUN/program/journal.jsonl" "$RUN/journal.jsonl"
    chmod -R a+rX "$RUN" 2>/dev/null
    ls -la "$RUN"
    ;;
write)
    cat > "$1"
    ;;
*)
    echo "usage: $0 preflight|env|start|check|stop|collect|write ..." >&2; exit 64
    ;;
esac
