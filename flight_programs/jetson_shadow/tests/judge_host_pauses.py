#!/usr/bin/env python3
"""LAPTOP TESTS ONLY: re-judge a run_shadow_test.sh FAIL against host/VM pauses the mock measured itself.

On the Windows laptop, WSL2 / Docker Desktop freezes every process in the VM for 0.4-1.5 s every
20-30 s (measured: mock_drone.py's own timers pause simultaneously in all four of its processes, with
no program running). A topic gap or a program stall that overlaps such a pause is the test host,
not the program. Everything else stays a FAIL. On the drone there is no mock, and nothing is excused.

    judge_host_pauses.py RUN_DIR MOCK_LOG    -> exit 0 if every failure is excused, else 1
"""
import json
import re
import sys
from pathlib import Path

run, mock_log = Path(sys.argv[1]), Path(sys.argv[2])
pauses = []                         # [start, end] wall seconds
for m in re.finditer(r"MOCK PAUSE ([0-9.]+) s at wall ([0-9.]+)", mock_log.read_text()):
    d, end = float(m.group(1)), float(m.group(2))
    pauses.append((end - d - .1, end + d + .2))  # WSL freezes the wall clock in a pause and steps it later
pauses.sort()


def covered(a, b):
    """[a, b] lies inside one measured pause (a gap LONGER than the pause is real: e.g. the injected one)."""
    return any(ps - .35 <= a and b <= pe + .35 for ps, pe in pauses)   # slack: one message period


def starts_in_pause(a):
    return any(ps - .3 <= a <= pe + .3 for ps, pe in pauses)


report = json.loads((run / "checker.json").read_text())
excused, real = [], []
for key, r in report["topics"].items():
    if r.get("verdict") != "FAIL":
        continue
    why = r.get("why", "")
    if "rate" in why or "no messages" in why or "never reached" in why or not r.get("gaps_over"):
        real.append(f"{key}: {why}")
        continue
    bad = [g for g in r["gaps_over"] if not covered(g[0], g[0] + g[1])]
    (real if bad else excused).append(f"{key}: gaps {r['gaps_over']}" + (f" NOT during a host pause: {bad}" if bad else ""))
other = [f for f in report["fails"] if not any(f.startswith(k + ":") for k in report["topics"])]
real += other

log_bad = []
for line in (run / "program.log").read_text().splitlines():
    m = re.match(r"\[(WARN|ERROR|FATAL)\] \[([0-9.]+)\]", line)
    if m:
        t = float(m.group(2))
        stall = re.search(r"stall ([0-9.]+) s", line)
        stale = re.search(r"stale input: \w+ ([0-9.]+) s old", line)
        ok = m.group(1) == "WARN" and ((stall and covered(t - float(stall.group(1)), t))
                                       or (stale and starts_in_pause(t - float(stale.group(1)))))
        (excused if ok else log_bad).append(line[:160])
    elif "Traceback" in line or "Error:" in line or "Warning:" in line:
        log_bad.append(line[:160])
real += [f"program.log: {line}" for line in log_bad]

print(f"host/VM pauses measured by the mock: {len(pauses)}, longest {max((b - a - .3 for a, b in pauses), default=0):.2f} s")
for e in excused:
    print(f"  excused (during a host pause): {e}")
for f in real:
    print(f"  REAL FAIL: {f}")
print("JUDGED:", "PASS (all failures were host pauses)" if not real else "FAIL")
sys.exit(1 if real else 0)
