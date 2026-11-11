#!/usr/bin/env bash
# End-to-end tests of run_shadow_test.sh --dry-run against mock_drone.py (NOT for the drone).
# In the Humble test image (Windows, Git Bash):
#   docker build -t jetson-shadow-humble -f jetson_shadow/tests/Dockerfile.humble jetson_shadow/tests
#   MSYS_NO_PATHCONV=1 docker run --rm -v "$(cygpath -w "$PWD/jetson_shadow")":/ws jetson-shadow-humble bash /ws/tests/e2e_humble.sh [unit|pass|gap|armed|all]
# Every process started here is stopped by its own PID.
set -u
set +u; . /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID=91 RCUTILS_COLORIZED_OUTPUT=0
export WS="${WS:-/ws}"
OUT=/tmp/e2e; mkdir -p "$OUT"   # container-local: a Windows bind mount (9P) blocks log writes for ~1 s
trap '[ -d /out ] && cp -r "$OUT"/. /out/' EXIT
what="${1:-all}"
rc_all=0

with_mock() {   # name, mock args..., -- , run_shadow_test args...
    local name="$1"; shift
    local margs=() targs=()
    while [ $# -gt 0 ] && [ "$1" != "--" ]; do margs+=("$1"); shift; done
    shift
    targs=("$@")
    echo; echo "################ e2e: $name  (mock: ${margs[*]:-none}; test: ${targs[*]:-none})"
    python3 -u "$WS/tests/mock_drone.py" "${margs[@]}" > "$OUT/mock-$name.log" 2>&1 &
    local mpid=$!
    sleep 3
    bash "$WS/run_shadow_test.sh" --dry-run --runs-dir "$OUT/runs-$name" "${targs[@]}"
    local rc=$?
    kill -TERM "$mpid" 2>/dev/null; wait "$mpid" 2>/dev/null   # TERM: background jobs ignore SIGINT
    local left; left="$(pgrep -f 'tests/mock_drone.py' || true)"
    if [ -n "$left" ]; then echo "HARNESS ERROR: mock processes survived: $left"; kill -KILL $left; rc_all=1; fi
    echo "################ e2e: $name -> run_shadow_test.sh exit $rc"
    grep -h "ODOMETRY GAP" "$OUT/mock-$name.log" || true
    LAST_RC=$rc
    LAST_JUDGE=-
    local run; run="$(ls -d "$OUT/runs-$name"/shadow-* 2>/dev/null | tail -1)"
    if [ "$rc" = 1 ] && [ -f "$run/checker.json" ]; then
        echo "---- laptop-only re-judge against host/VM pauses measured by the mock:"
        python3 "$WS/tests/judge_host_pauses.py" "$run" "$OUT/mock-$name.log"; LAST_JUDGE=$?
    fi
}

if [ "$what" = unit ] || [ "$what" = all ]; then
    echo "################ versions"; python3 -c "import sys, numpy; print('python', sys.version.split()[0], 'numpy', numpy.__version__)"
    python3 -c "import scipy" 2>/dev/null && echo "scipy present" || echo "scipy: not installed (fallback path)"
    echo "ROS_DISTRO=$ROS_DISTRO"
    (cd /tmp && python3 "$WS/tests/test_shadow.py" 2>&1 | sed 's/^/  /'; exit "${PIPESTATUS[0]}") || rc_all=1
    python3 - <<'EOF' || rc_all=1
import sys, re, ast
# Python <= 3.10 syntax: the file must compile (3.10 here); and nothing needs numpy > 1.21
src = open(__import__("os").environ.get("WS", "/ws") + "/upload_to_flyx_v4_skybeta.py").read()
compile(src, "x", "exec")
newer = [n for n in ("np.char.", "numpy.typing", "np.random.Generator.spawn", "np.exceptions", "np.trapezoid", "np.bool(", "np.long", "np.concat(", "np.unstack", "np.cumulative_sum", "np.astype(")
         if n in src]
print("numpy >1.21-only API used:", newer or "none")
sys.exit(1 if newer else 0)
EOF
fi
if [ "$what" = pass ] || [ "$what" = all ]; then
    with_mock pass -- --duration 60
    [ "$LAST_RC" = 0 ] || [ "$LAST_JUDGE" = 0 ] || rc_all=1      # PASS, or FAIL only from host pauses
fi
if [ "$what" = gap ] || [ "$what" = all ]; then
    # 35 s after the program first publishes status: well inside the checker window
    with_mock gap --odom-gap-after-status --odom-gap-at 35 --odom-gap-s 3 -- --duration 60
    [ "$LAST_RC" = 1 ] && [ "$LAST_JUDGE" = 1 ] || rc_all=1      # must FAIL, and not be excusable
fi
if [ "$what" = armed ] || [ "$what" = all ]; then
    with_mock armed --armed -- --duration 60
    [ "$LAST_RC" = 2 ] || rc_all=1          # must REFUSE
    if pgrep -f upload_to_flyx_v4_skybeta.py >/dev/null; then echo "program was started despite armed!"; rc_all=1; fi
fi
echo; echo "################ e2e overall: $([ $rc_all = 0 ] && echo OK || echo PROBLEM)"
exit $rc_all
