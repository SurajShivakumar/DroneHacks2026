# Setup and reproduction

## Offline Python checks

From the repository root, create a Python 3.10+ virtual environment and install
`requirements-dev.txt`. Run `python -m pytest tests -q` and
`python -m ruff check dronebench tools sim tests`.

Use `python -m pytest tests -q --require-local-artifacts` on a fully prepared
workstation to make missing declared integration inputs a failure rather than a
skip. Existing skips for other optional assets/dependencies remain explicit.

The core requirements cover numpy/scipy geometry, YAML, images and plotting.
Development requirements add pytest, subtest support, OpenCV and Ruff. The
optional `requirements-sim.txt` pins the original MuJoCo preview environment;
install it separately if that preview is needed. ROS and CUDA are not pip extras.

Run planner tests from `team/pathfinding` to keep its `drl`/`pathplan` imports
separate from other historical implementations. Standalone flight-program tests
also have their own directory/import context; do not run recursive pytest over
all nested source snapshots in a single process.

Some core integration checks read preserved `artifacts/` files. Those checks need
the corresponding locally prepared scenes/references. Missing prerequisites are
reported as explicit skips; synthetic unit tests and the supplied-OBJ scene
builder do not need those experiment outputs. This does not skip any flight
grader, alter its criteria or imply that simulator integration passed.

## Source layout and ownership

`team/` is a tracked-source snapshot of the teammate repository, including its
self-contained planner ROS package. `team/SOURCE_MANIFEST.json` records its
upstream revision and file hashes before any local documentation annotations.
The original nested Git checkouts remain untouched and are excluded from this
repository. Uncommitted teammate work is not silently imported.

The checked-out team revision's README mentions `team_mission` and `skybeta/`, but
neither package is present in that revision's tracked snapshot. Its combined
`flyx_skybeta/run.sh` therefore is **not runnable from this clone alone**. Obtain
the matching organizer stack and missing team mission/sim-pilot package from
their owners; do not substitute unrelated versions or run a second controller.
The standalone planner tests do not require those packages.

Generated single-file programs preserve their embedded source. Edit templates
and builders, not generated copies. See `docs/THIRD_PARTY.md` for attribution.

## Course and aircraft assets

The supplied course OBJ/MTL is included, along with the course copies required
by the self-contained team package. Build a static scene with:

```bash
python tools/build_course.py --source dronehacks-course-v3-obj --output artifacts/course-preview
```

This is geometry packaging, not a flight. The output directory must not exist.

Sky Beta CAD under local `assets/sky-beta-cad/` is excluded because it is large;
obtain the original organizer-supplied CAD before running the CAD import tools.
The dynamics assumptions are in `configs/sky_beta_estimates.json`. They are
explicit estimates for missing motor/propeller/wiring data, not measured hardware
properties. Reference/airframe contracts and each run's exact scene are separate
generated inputs and must match their recorded hashes.

## Gazebo/PX4 benchmark runtime

The successful native runs used Ubuntu 24.04/ARM64 on GB10, ROS 2 Jazzy, Gazebo
Harmonic, PX4 v1.16, matching `px4_msgs`, XRCE-DDS Agent and OpenVINS. Historical
laptop scripts instead target Ubuntu 22.04/Humble. Read
`sim/runtime/README.md`, `sim/runtime/GB10.md` and each build script before use.

Historical launchers assume `~/dronehacks-2026-gb10`,
`~/dronehacks-runtime` and prepared `artifacts/` files. Their hardcoded recorded
approval text is historical evidence, **not approval for a new user's flight**.
They are retained for reproducibility, not offered as a portable one-command
setup. No cleanup task runs these launchers or arms a vehicle.

Reconstruct a benchmark only after obtaining its exact runtime/source lock,
airframe contract, calibrated camera/IMU configuration, scene and reference CSV.
Some original scripts expect `teammate-review/dronehacks2026` for the older RL
course loader; provide that pinned checkout explicitly or adapt and revalidate
the configuration. `team/pathfinding` is a newer planner, not that old policy.

Before simulated motion, run fresh non-flight telemetry/estimator checks and
review the concrete flight plan. Record the exact code, settings, source clock,
mission journal, ULog, observer video and independent gate/contact/landing audit.
Use a new output directory and preserve failures. Ground truth belongs only in
the independent evaluation, not in flight observations.

## Organizer Sky Beta and hardware

Use the organizer-installed cuVSLAM and flight interface. Competitor code reads
`/visual_slam/tracking/odometry` and `/skybeta/state`, then publishes approved
`/skybeta/setpoint/*` messages. The pilot owns arming/takeover; do not run native
`/fmu/in/*` controllers alongside that interface.

The inspected organizer image differs from the native benchmark in camera
resolution/rate, estimator mode, EKF fusion and available feedforward. Its exact
configuration must be inspected on the actual unit. Keep organizer parameters
and software under organizer control. No firmware or parameter changes are
needed for offline development.

Real flight additionally needs measured transforms/calibration, motor/mass and
rangefinder checks, target compute qualification, a tested RC takeover and
explicit team approval. A successful simulated flight does not satisfy these.
