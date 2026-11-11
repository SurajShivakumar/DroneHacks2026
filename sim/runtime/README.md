# Local Gazebo/PX4 proxy runtime

These scripts operate in the laptop's **Ubuntu-22.04 WSL distribution**, not the
managed companion lab. They do not install anything on the issued drone. The
chosen proxy is Ubuntu 22.04, ROS Humble, Gazebo Harmonic and PX4 v1.16.0.
It is not a claim that these are the organizer's final versions or airframe.
Humble/Harmonic uses OSRF's non-default `ros-humble-ros-gzharmonic` packages;
do not mix in the Fortress `ros-humble-ros-gz` package family.

## Setup and build

Run from this repository in Windows PowerShell. Logs are exclusive-created;
choose a fresh log filename on retries.

```powershell
python tools/wsl_run.py sim/runtime/install_dependencies.sh --user root --log artifacts/runtime/install-dependencies-01.log
python tools/wsl_run.py sim/runtime/fetch_sources.sh --log artifacts/runtime/fetch-sources-01.log
python tools/wsl_run.py sim/runtime/build_px4.sh --log artifacts/runtime/build-px4-02.log
python tools/wsl_run.py sim/runtime/build_ros.sh --log artifacts/runtime/build-ros-01.log
python tools/wsl_run.py sim/runtime/prepare_proxy.sh --log artifacts/runtime/prepare-proxy-01.log
```

The dependency script adds signed official ROS/OSRF apt repositories and installs
packages in that WSL distribution. No wholesale OS upgrade is requested. Source
and native Linux builds live in `~/dronehacks-runtime`. PX4 Python dependencies
are isolated in `px4-venv`; the agent installs to a user-owned prefix. Neither
build script starts an autopilot. Matching message branch is `release/1.16`;
exact checked-out commits are retained in `source-lock.json` and copied into
the workspace's `artifacts/runtime/runtime-lock.json` with package/binary hashes.
Uninitialized unrelated PX4 submodules are intentional for a SITL-only build.

`wsl_run.py` sends LF bytes rather than Windows text-mode CRLF, preserves the
whole combined log and prints Unicode safely. A first PX4 attempt was stopped
by the helper's cp1252 progress-bar printing failure; it is retained in
`build-px4-01.log`. The helper was corrected and PX4 built successfully on retry.

## No-GNSS proxy scene

`sim/gazebo_proxy.py` copies the pinned upstream X500 base/motor model into the
course world. It removes the NavSat sensor from the copy, preserves upstream
motor dynamics and leaves upstream source files unchanged. It adds illustrative
640x400/15 Hz stereo and RGB cameras, ideal rendered depth and a single-beam
downward scan. It does not add a truth-odometry publisher or a position fix.
Camera extrinsics, optical frames, IMU rate/noise and range response require
validation before localization. X500 dynamics are not qualified Sky Beta dynamics.

## Bounded non-flight checks

```powershell
python tools/wsl_run.py sim/runtime/sensor_smoke.sh --log artifacts/runtime/sensor-smoke-01.log
python tools/wsl_run.py sim/runtime/disarmed_smoke.sh --log artifacts/runtime/disarmed-smoke-01.log
```

`sensor_smoke.sh` runs Gazebo and a GZ-to-ROS sensor bridge, observes actual
messages for 30 wall seconds, and stops all children. `disarmed_smoke.sh` also
starts the compiled PX4 and exactly one XRCE agent. Each check uses an isolated
Gazebo partition and ROS domain 42. The PX4 check disables GNSS fusion at startup
for the explicitly no-GNSS proxy; it does not alter safety/failsafe limits or add
external-vision data. Every run uses a new rootfs/parameter store.

No script publishes flight setpoints, offboard heartbeats, mode changes or arm
commands. The observer aborts on an armed status. This is not flight qualification:
without a validated estimator the vehicle is expected to lack an XY position fix.
The node reports wall-clock receive rate separately from source stamp progression,
retains estimator flags and writes nonfinite values explicitly as strings.

Results are under `artifacts/gazebo-sensor-smoke-v1/` and
`artifacts/gazebo-px4-smoke-v1/`. Pick fresh output paths inside a reviewed wrapper
for a retry; old logs and failures are never overwritten. All smoke-test processes
are stopped on success or error. See ../../docs/BENCHMARKS.md for observed outcomes rather
than assuming that script availability means a passing run.

## Next integration work

1. Verify camera optical TF/CameraInfo, stereo synchronization and sensor latency.
2. Measure unloaded real-time factor; reliable image QoS resolved the earlier
   RGB/depth delivery loss in the v8 sensor check.
3. Validate OpenVINS in shadow mode, then review any external-vision integration.
4. Connect observed mapping, height-aware planning, mission state and authority gate.
5. Propose a bounded takeoff/hover/land plan for team approval, then qualify stages.

The new `dronebench/grid.py` is a conservative A* component over an observed
occupancy slice. Unknown cells and insufficient footprint clearance are blocked;
grid origin rotation is supported. It currently returns cell-center paths and
requires separate connector/smoothing checks. It is not a complete 3D planner,
map builder or live flight node.
