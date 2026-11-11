# Getting started

Read the [project README](../README.md) for the challenge, current architecture,
results and component map. For dependencies, runtime differences and missing
assets, use [REPRODUCING.md](REPRODUCING.md).

## Offline checks

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\Activate.ps1
python -m pip install -r requirements-dev.txt
python -m pytest tests -q
python -m ruff check dronebench tools sim tests
```

Run the online planner tests separately:

```bash
cd team/pathfinding
python -m pytest tests -q
```

No command above arms a vehicle. Simulator integration and hardware flight require
separate runtime setup, fresh telemetry and approval for the concrete mission.

Selected reports are in [benchmarks](../benchmarks/README.md); recordings and raw
logs stay outside Git. A nominal reference time is not a completed flight time.

The old November 6 introduction is preserved under
[history](history/GETTING_STARTED-2026-11-06.md). Its machine paths, launch commands
and status are historical, not a fresh-clone setup guide.
