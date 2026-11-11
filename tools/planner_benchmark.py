"""CPU A*/shortcut benchmark on explicit synthetic observed grids, not flight.

Run: python -m tools.planner_benchmark --output artifacts/planner-benchmark-v1
No scene geometry, simulator truth, ROS topics or flight commands are consumed.
"""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
from hashlib import sha256
import json
from math import ceil
from pathlib import Path
import platform
from statistics import median
import sys
from time import perf_counter_ns

from dronebench.grid import OccupancySlice, find_path
from dronebench.path_smoothing import LineOfSight, path_length, shortcut_path


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode()


def fixtures():
    # These are deliberately authored observed maps, not maps of the race scene.
    width, height = 90, 30
    cells = tuple(100 if row in (0, height - 1) else 0
                  for row in range(height) for col in range(width))
    yield "synthetic_straight_corridor", OccupancySlice(width, height, .1, cells), (8, 15), (80, 15), True
    width, height = 90, 60
    cells = tuple(100 if (row in (0, height - 1) or col in (0, width - 1)
                         or (col == 30 and row < 43) or (col == 60 and row > 16)) else 0
                  for row in range(height) for col in range(width))
    yield "synthetic_alternating_maze", OccupancySlice(width, height, .1, cells), (8, 8), (80, 50), True
    cells = tuple(-1 if col == 45 else 0 for row in range(30) for col in range(90))
    yield "synthetic_unknown_barrier", OccupancySlice(90, 30, .1, cells), (8, 15), (80, 15), False


def summary(values):
    ordered = sorted(values)
    return {"median_ms": median(ordered) / 1e6,
            "p95_ms": ordered[ceil(.95 * len(ordered)) - 1] / 1e6}


def run(output, repetitions=30, warmup=3, radius=.15):
    if output.exists():
        raise FileExistsError("choose a fresh output directory; preserve earlier evidence")
    if repetitions < 2 or warmup < 0:
        raise ValueError("at least two repetitions and nonnegative warmup required")
    output.mkdir(parents=True)
    root = Path(__file__).resolve().parents[1]
    sources = [Path(__file__).resolve(), root / "dronebench/grid.py",
               root / "dronebench/path_smoothing.py"]
    result = {"kind": "synthetic_observed_grid_cpu_component_benchmark",
              "created_utc": datetime.now(timezone.utc).isoformat(),
              "not_course_flight_or_finish_time": True,
              "trajectory_dynamically_validated": False,
              "settings": {"radius_m": radius, "occupied_threshold": 50,
                           "repetitions": repetitions, "warmup": warmup,
                           "p95_method": "nearest rank", "concurrency": "single process"},
              "runtime": {"python": sys.version, "platform": platform.platform(),
                          "processor": platform.processor()},
              "sources": [{"path": str(p.relative_to(root)),
                           "sha256": sha256(p.read_bytes()).hexdigest()} for p in sources],
              "cases": []}
    result["settings_sha256"] = sha256(encoded(result["settings"])).hexdigest()
    for name, grid, start_cell, goal_cell, expected in fixtures():
        start, goal = grid.cell_to_world(*start_cell), grid.cell_to_world(*goal_cell)
        case = {"name": name, "input_grid": asdict(grid), "start": start, "goal": goal,
                "expected_reachable": expected, "runs": []}
        case["input_sha256"] = sha256(encoded({"grid": asdict(grid), "start": start, "goal": goal})).hexdigest()
        for index in range(warmup + repetitions):
            begin = perf_counter_ns()
            path = find_path(grid, start, goal, radius)
            astar_end = perf_counter_ns()
            shortened = shortcut_path(grid, path, radius) if path else None
            shortcut_end = perf_counter_ns()
            # Validation is deliberately outside the timed section.
            if bool(path) != expected:
                raise AssertionError(f"{name}: unexpected reachability")
            if shortened:
                checker = LineOfSight(grid, radius)
                assert all(checker.clear(a, b) for a, b in zip(shortened, shortened[1:]))
                assert path_length(shortened) <= path_length(path) + 1e-8
            if index >= warmup:
                case["runs"].append({"astar_ns": astar_end - begin,
                                     "shortcut_ns": shortcut_end - astar_end,
                                     "total_ns": shortcut_end - begin,
                                     "success": bool(path)})
        case["reachable_runs"] = sum(item["success"] for item in case["runs"])
        case["timing"] = {key: summary([item[key + "_ns"] for item in case["runs"]])
                          for key in ("astar", "shortcut", "total")}
        case["astar_path"], case["shortened_path"] = path, shortened
        case["astar_length_m"] = path_length(path) if path else None
        case["shortened_length_m"] = path_length(shortened) if shortened else None
        result["cases"].append(case)
    report = output / "result.json"
    report.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    (output / "evidence-sha256.json").write_text(json.dumps({"result.json": sha256(report.read_bytes()).hexdigest()}, indent=2) + "\n")
    for case in result["cases"]:
        print(case["name"], case["reachable_runs"], "/", repetitions,
              "total timing", case["timing"]["total"], "length_m", case["shortened_length_m"])
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=30)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--radius", type=float, default=.15)
    args = parser.parse_args()
    run(args.output, args.repetitions, args.warmup, args.radius)
