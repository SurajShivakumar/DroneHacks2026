# Repository cleanup validation

Date: 2026-11-10. This work prepares a source repository; it does not run a flight,
change PX4 parameters or establish new hardware qualification.

## Scope

- Replaced the stale root README with challenge, full architecture, measured
  results, control contracts, setup and remaining work.
- Added a current getting-started guide; preserved its older version in history.
- Imported a tracked-source teammate snapshot with original and published hashes;
  preserved headers and documented missing organizer/mission dependencies.
- Added Python dependency manifests, offline CI, narrow undefined-name lint checks,
  line-ending rules and exclusion of private/generated/heavy local files.
- Copied selected existing benchmark reports/evaluations with original hashes.
- Removed one unused `hypot` import from the pure teammate-policy wrapper.
- Corrected stale CLI strategy metadata to distinguish recorded native completion
  and offline model inference from unqualified flight adapters; dated upstream
  model-release status rather than claiming it was rechecked during cleanup.
- Declared prerequisites for artifact-dependent integration tests. Their
  assertions remain intact, and strict mode fails if those inputs are missing.
- Preserved local raw evidence, failures, working checkouts, CAD, archives and
  historical scheduler files; those are excluded from this source release.

## Checks performed

All commands ran without arming or publishing flight controls, using separate
Python processes where package names overlap.

| Check | Outcome |
|---|---|
| Initial `python -m pytest tests -q` | 360 passed, 19 skipped, 52 subtests passed |
| After cleanup `python -m pytest tests -q --require-local-artifacts` | 360 passed, 19 skipped, 52 subtests passed; same existing optional skips |
| `team/pathfinding`: `python -m pytest tests -q` | 134 passed |
| `team/suraj_course`: `python -m pytest -q test_suraj_course.py` | 15 passed |
| `flight_programs/flyx_course`: `python -m pytest -q test_fly_course.py` | 15 passed |
| `flight_programs/flyx_square`: `python -m pytest -q test_fly_square.py` | 13 passed |
| `team/path_recorder`: `python -m pytest -q test` | 8 passed |
| `team/path_player`: `python -m pytest -q test` | 9 passed |
| `python -m ruff check dronebench tools sim tests` | Passed F821/F822/F823 correctness checks |
| Staged snapshot and evidence hash verification | All matched |
| Staged-file inventory and credential-pattern scan | No unexpected local/generated files or recognized credential patterns found |
| Artifact-free export using the original environment | 343 passed, 36 skipped, 52 subtests passed |
| Artifact-free export in a new isolated environment | 359 passed, 20 skipped, 52 subtests passed before adding the optional FFmpeg wheel below |
| Final artifact-free export with all declared development dependencies | **360 passed, 19 skipped, 52 subtests passed**; OpenCV and recording checks executed |
| Strict artifact-mode negative check in the export | Correctly failed on the deliberately absent declared input |
| Isolated dependency installation, `pip check` and Ruff | Passed; no inherited system packages required |
| Hosted Ubuntu/Python 3.11 CI on initial commit `db44d69` | Both core and team-planner/component jobs passed: [run](https://github.com/SurajShivakumar/DroneHacks2026/actions/runs/42955276869) |

The pre-existing environment is Windows/Python 3.13 and inherits some installed
packages. A separate clean-source export was also tested without local artifacts;
isolated dependency installation and hosted Linux CI are checked separately below.
Pattern scanning does not prove absence of every possible sensitive value.

The first isolated run exposed an additional optional recording-test skip because
`imageio-ffmpeg` was missing from the development installation. It is now declared
in the core requirements so recording support can be exercised from a fresh clone.

## Fresh-clone correction

The first artifact-free export exposed 13 failures caused by hardcoded local
`artifacts/` prerequisites, rather than changed algorithms. Those test cases now
declare their required files via `local_artifacts(...)`. If the declared files
are absent they explain the skip; if present the original assertions run. Use
`--require-local-artifacts` to fail on missing declared inputs. This does not
modify any flight grader or treat skipped integration checks as passing flights.

Other existing optional checks still require prepared MuJoCo scenes, recordings,
external assets or ROS. Full simulator reproduction needs the pinned runtime,
calibration, generated scenes/airframe contract and reference, as described in
[REPRODUCING.md](REPRODUCING.md). Historical GB10 launch scripts are not a complete
fresh-clone installer.
