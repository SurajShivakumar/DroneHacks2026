# sim_stack: simulated Sky Beta course baseline (start here)

**Best result so far: complete course in 119.56 s arm→disarm** (run v55, `artifacts/gb10-sky-beta-course-v55/`; previous best v51 146.42 s,
Gazebo Harmonic + PX4 v1.16 SITL + OpenVINS, estimated Sky Beta). Takeoff →
all 9 ordered gates → pad landing/disarm, zero structure contacts by
independent truth evaluation.

- Watch it: [`artifacts/gb10-sky-beta-course-v55/flight-video/flight.mp4`](artifacts/gb10-sky-beta-course-v55/flight-video/flight.mp4)
  (4× version: `flight-4x.mp4`). Top = overhead observer, bottom = onboard stereo + VIO tracks.
- Numbers: [`artifacts/gb10-sky-beta-course-v55/README.md`](artifacts/gb10-sky-beta-course-v55/README.md); full pipeline explanation: [`artifacts/gb10-sky-beta-course-v51/README.md`](artifacts/gb10-sky-beta-course-v51/README.md)
- Proposed next architecture: [`ARCHITECTURE_PLAN.md`](ARCHITECTURE_PLAN.md)
- Earlier slow (498 s) first completion: `artifacts/gb10-sky-beta-first-completion-v40/README.md`

## Results so far (simulated, n small)

| Run | Reference / control | Outcome | Arm→disarm |
|---|---|---|---:|
| v40 | in-place turns, 0.15 rad/s | complete | 498.5 s |
| v41 | v40 repeat | VIO feature loss turning at tube mouth | — |
| v44, v50 | continuous k08, no feedforward | complete ×2 | 182.6, 183.8 s |
| v45–v49 | k09/k10/k12/k16, no feedforward | overshoot at tube-mouth stop (4 cm guard) | — |
| **v51** | **k12 + accel/yaw feedforward** | **complete** | **146.4 s** |
| v53 | k08 + feedforward | complete | 150.3 s |
| v52 | k10 + feedforward | VIO feature dropout at 35 s | — |
| v54, v56 | fast f5 / f9 (no kink stops) | VIO feature loss near corner1 | — |
| **v55** | **fast f5 (1.2 m/s cap, 40 cm smoothing)** | **complete** | **119.6 s** |

## Layout

- `tools/prepare_fast_reference.py` — the planner: published-map path →
  perception-aware heading DP → speed profile (accel/yaw/kink limits).
- `tools/course_flight.py` — the onboard mission supervisor (ROS 2/px4_msgs):
  preflight, arm, takeoff, reference tracking with guards, land, disarm.
  `acceleration_feedforward` / `yawspeed_feedforward` settings flags are new.
- `dronebench/` — shared library (frames, tracking clock, VIO watchdog, audits).
- `configs/sky_beta_course_continuous_k*_ff.json` — mission settings.
- `sim/` — Gazebo world/airframe/rig builders and `gazebo_smoke.py` (runner).
- `sim/runtime/gb10_sky_beta_course_run.sh` — one simulated flight;
  `gb10_slot_run.sh` runs it in an isolated namespace so 3 can run in parallel.
- `tools/evaluate_flight_truth.py` — after-flight truth evaluation (contacts,
  landing, tilt-aware gates). `tools/encode_flight_video.py` — recordings.
- `tests/` — `python -m pytest -q tests` (342 pass on Windows with MuJoCo).

`teammate-review/dronehacks2026` (expected by the scripts) is a symlink to the
repository root, i.e. this repo's PPO code/config/course files.

## Runtime (GB10 / WSL2 Ubuntu 24.04)

ROS 2 Jazzy, Gazebo Harmonic (ros-jazzy-ros-gz), PX4 v1.16.0
(`6ea35391…`), px4_msgs `392e831…`, Micro-XRCE-DDS-Agent v2.4.3, OpenVINS
`69488123…` with the two patches in `sim/runtime/fix_gb10_vio_*.sh`, QGC 5.1.5.
Build scripts: `sim/runtime/build_gb10_*.sh`. Paths expected:
`~/dronehacks-2026-gb10` (this folder) and `~/dronehacks-runtime`.

```bash
bash sim/runtime/gb10_sky_beta_course_run.sh \
  artifacts/published-course-continuous-k12/trajectory.csv \
  configs/sky_beta_course_continuous_k12_ff.json \
  artifacts/my-run-v1
```

This arms a *simulated* aircraft; follow the team approval rule in `CLAUDE.md`.

## Known gaps / good next tasks

- Success rate: repeat k12+ff / k08+ff runs (only one each so far).
- Organizer lab uses **cuVSLAM**, not OpenVINS — port the estimator path.
- Online SLAM/mapping (encouraged): stereo-depth local map to check the
  corridor ahead and correct drift (building blocks in `dronebench/depth_*`,
  `wall_fusion.py`, `map_vio.py`).
- Scene uses estimated wall textures; real venue appearance is unverified.
- VLA: OpenFly's released 3 m / 30° macro actions are unsafe in 1 m corridors
  (3 m forward step collision-free from 2/60 route poses); open-loop scorer in
  `tools/vla_route_agreement.py`.
