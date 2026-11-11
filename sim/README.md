# Local simulator integration

## Primary flight benchmark target: Gazebo + PX4

Match the organizer's actual Gazebo/PX4/ROS starter stack for end-to-end mission
tests and course timing. PX4 documents Gazebo SITL and ROS 2 integration:
https://docs.px4.io/main/en/sim_gazebo_gz/ . The local Harmonic export is a prepared
world, not a pinned event runtime. Match actual versions for event qualification.
The local proxy is now installed: Ubuntu 22.04, Humble, Harmonic 8.15.0 and
PX4 v1.16.0, with exact source/package hashes under artifacts/runtime/.
See [runtime commands and limitations](runtime/README.md).
Gazebo Classic 11 remains only a compatibility smoke-check environment here.

OBJ/MTL files are simulator-neutral; supported aircraft, sensor and PX4 interfaces
determine this choice. Use headless simulation with required onboard camera
streams and optional observer recording to manage laptop load. Integration and
performance still need measurement. The existing MuJoCo proxy is preserved for
fast isolated experiments, with separate results from Gazebo/PX4 benchmarks.

## Working optional sandbox: MuJoCo

MuJoCo 3.16.0 runs and renders natively on this Windows laptop. No Isaac Sim,
Docker, WSL or ROS installation is required for the local scene/physics tools.
Use the project `.venv`; its initial setup inherited existing Python packages
with `--system-site-packages` and added CoACD. Other machines can create an
isolated environment and install `requirements-sim.txt` directly.

```powershell
python -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements-sim.txt
.\.venv\Scripts\python -m sim.mujoco_course --output artifacts/mujoco-course-v1
.\.venv\Scripts\python -m sim.mujoco_preview artifacts/mujoco-course-v1/course.xml --output artifacts/mujoco-preview-v1
.\.venv\Scripts\python -m unittest discover -s tests -v
.\.venv\Scripts\python -m mujoco.viewer --mjcf=artifacts/mujoco-course-v1/course.xml
```

Choose fresh output paths to preserve previous builds and recordings. Scene build
takes longer than preview rendering because it decomposes concave geometry.
The optional MuJoCo tests skip if the package or default generated scene is absent;
the original offline planning/Gazebo-package tests do not require MuJoCo.

The importer preserves the original 33 mesh groups and diffuse colors. Solid
collision objects use convex pieces; CoACD handles tunnel/window/cradle concavity.
A single convex hull would incorrectly fill those holes. The requested CoACD
threshold is 0.01 m; this is an approximation setting, not a clearance guarantee.
Decorative floor lines and pad markings have no separate colliders; the imported
floor supports the drone. Net visuals are transparent and net collisions remain
solid thin-sheet approximations. MuJoCo contacts and visual appearance are not
physical netting qualification.

The generic quad has a free rigid body, four body-local thrust/torque actuators,
RGB/stereo camera definitions, gyro, accelerometer and downward rangefinder.
Mass/inertia/rotor envelopes, 75 mm stereo baseline and camera mounts are illustrative.
No motor lag, propeller aerodynamics, PX4 control, pose fix, VIO, ROS transport,
or VLA adapter exists yet. Camera images/renderer depth are idealized; do not call
the depth output reconstructed OAK depth. Do not send raw MuJoCo state into the
future flight algorithm as a substitute estimator.

Preview output includes an MP4 with an explicit no-flight label, observer PNG,
onboard RGB PNG, ideal-depth `.npy`, and timing metadata. The frozen camera orbit
advances no physics and sends no actuator commands. The viewer does advance physics
but has no flight controller; the proxy quad can settle under gravity.

Measured scene-only rendering: ~21.4 frames/s at 960x540, with 72 frames recorded
in ~3.37 s. All 15 tests passed including hole/contact and gravity-balance probes.
These rates exclude VLA inference/VIO and are not mission finish times.

## Gazebo export for the primary integration target

The downloaded OBJ and MTL define the course, not a flying drone simulation.
`tools/build_course.py` packages them as a static Gazebo Harmonic model/world.
The generated collision mesh includes all supplied geometry, including the net;
contact behavior, materials, lighting and narrow openings require runtime inspection.
The simple light is a placeholder: tunnel photometry is NOT qualified by this scene.

Build from the repository root on Windows or Linux:

```text
python tools/build_course.py
```

On a Linux environment with **Gazebo Harmonic installed**, from the repo root:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/artifacts/course-v3/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim artifacts/course-v3/course.sdf
```

These commands open only a static scene. No drone is spawned or armed. The world
sets a requested physics step of 1/240 s; actual rates must be measured.
Do not add or run these host setup commands inside the managed companion lab.

## Missing prerequisites for a flight benchmark

1. A Linux Gazebo/PX4/ROS 2 installation with mutually compatible pinned versions.
2. The organizer's Sky Beta dynamics, dimensions, camera/IMU calibration and starter
   mission source. A substitute airframe must be labeled as a proxy benchmark.
3. Stereo/RGB/depth/IMU/range sensors and measured delivery/latency/noise contracts.
4. Standard ROS bridges, `/clock`, matching `px4_msgs`, external-vision estimator
   and reviewed transport. Validate localization without using simulator truth.
5. An independent checkpoint/contact evaluator and observer/camera recordings.
6. End-to-end planner, model-specific inference adapters and flight authority gate.

None of these is provided by the static model. A scene load is not a mission pass.
The geometry summary is a published-layout prior, never a real-time sensor.

## Host inspection (2026-11-01)

- Windows Python 3.13 is available.
- WSL distributions include Ubuntu 22.04, Ubuntu, Ubuntu 20.04 and Docker Desktop.
- Inspected Ubuntu 22.04: Python 3.10; no `gz`, `ros2`, `colcon`, `/opt/ros`,
  or `~/PX4-Autopilot` found by the initial probe.
- Docker CLI exists but its Linux engine was not reachable.
- `nvidia-smi` returned insufficient permissions; usable GPU acceleration is unknown.
- FFmpeg is available on Windows.
- Additional inspection found Gazebo Classic 11 and ROS Noetic in Ubuntu 20.04.
  This is an older ROS 1 environment, not the target ROS 2/Harmonic runtime.

The packager also generates `course-classic.world` for geometry inspection on
that existing installation. Set `GAZEBO_MODEL_PATH` to the generated `models`
directory before opening it with Gazebo Classic. This is a separate compatibility
scene, not evidence that the target flight stack runs.

A 12-second server smoke check reached loading the generated Classic world;
see ../docs/BENCHMARKS.md for the preserved first failure, correction and raw logs.
Model insertion, collision behavior and rendering remain unverified.

The list above describes the initial inspection. Local proxy dependencies and
builds have since been installed in Ubuntu-22.04 WSL; Gazebo/ROS sensor checks
have run. No autonomous mission or arming command has been executed.
