# Baseline benchmark status

Full-turn shadow check v6 is also unqualified: 909 independently evaluated
frames, filtered XY 9.20 mm RMS / 103.30 mm max versus raw VIO 6.55 mm /
30.16 mm. The added filter overshot a braking transient; it remains disconnected
from PX4. Its actual fixture video and failure evidence are under
`artifacts/live-stereo-shadow-v6/`. Native feature health still fails.

Offline return geometry now passes the strict 8 cm margin screen at a 2.10 m
transit height, with 27.05 s hover-to-hover reference duration. This excludes
rearm/takeoff/landing. Fresh native qualification adds 60 s, so the earlier
40 s total-return reserve is infeasible under current policy. No return flight
or 180 s complete-mission result is claimed.

Live shadow update: v5 processed1,302 actual GB10 frames with no fatal observer
error. Over source15..75s, total processing median25.22ms/p9540.79ms/max45.51ms.
Filtered XY RMS9.54mm/max46.23mm versus raw VIO8.64mm/max24.96mm; this easy-view
trial does not demonstrate an accuracy gain or airborne qualification. The
native feature guard still fails. Evidence/video:
`artifacts/live-stereo-shadow-v5/`. v4's fixed-yaw initialization failed0.804m
RMS; v5 calibrates the independently chosen VIO yaw axes against the explicitly
declared initial physical heading. All failures remain preserved.

Offline route preparation now has an89.9626s candidate on the unchanged public
A* path, with bounded acceleration/braking, no stationary yaw spins, zero
strict mesh penetrations at8cm margin and all9 diagnostic ordered planes
passed. `artifacts/published-course-efficient-v1/` is reference geometry/timing,
not a flown course or official score. Actual takeoff/landing/return timing and
fixed-heading camera visibility remain unverified. Full suite:334 tests and
52 subtests pass.

Latest November 5 status: **no verified full-course finish**. The bounded
corner qualification launched and safely aborted/landed on the first straight
when the unchanged feature watchdog detected track loss. Observer flight clip:
`artifacts/gb10-sky-beta-corner-qualification-v1/flight-only-video/preview.mp4`.
Ground-height fixture passes below do not qualify flight-height estimation.

Actual stereo plus public-map wall factors reached **8.82 mm XY RMS / 16.96 mm
maximum** on 29 small-motion recorded frames, with **44.3 ms median** sensor
processing on the laptop. This is an offline local diagnostic, with fixed
height and engineering covariance assumptions; no flight odometry was created.
The failed velocity-only predecessor remains recorded (12.1 cm RMS).
See `artifacts/stereo-kinematic-active-view-v2/` and `PLAN.md` for the current
live shadow validation work. Local verification: **325 tests / 52 subtests**.

Sky Beta has completed bounded Gazebo/PX4 hover and landing trials. Course
finish times remain empty. The latest A* stop/turn/go trial did not finish and
is not a speed result: scene-only review found contact at the north net around
127.225 s, then the estimator/controller tracking guard requested Land at
128.471 s. PX4 confirmed landing and disarm. The teammate PPO
now has reproduced NumPy-simulator rollouts and video; these end at pad hover.
OpenFly BF16 offline inference now has a complete GB10 record: median 0.741909 s
over three measured samples after warm-up. This is inference latency, not flight
time; see [model results](MODEL_BENCHMARKS.md).

The read-only ROS route observer now consumes actual OpenVINS SLAM feature
clouds. Its post-fix disarmed GB10 rail-motion capture reached READY and ran
without a watchdog fault (361 callbacks; source age at most 4.17 ms; no flight
commands). A false future-stamp fault in the preceding run was traced to
floating-point epsilon and corrected with a 1 microsecond representation
tolerance, leaving the 0.5 s stale-data limit intact. This is diagnostic
readiness evidence, not an airborne course result; see
`artifacts/gb10-sky-beta-vio-quality-v9/` and
`artifacts/gb10-sky-beta-vio-quality-v10/`.

The A* run's native estimator drifted by approximately 0.67 m along the course
axis relative to the independent Gazebo pose before abort. OpenVINS logged zero
MSCKF features in 837 of 928 update cycles (maximum 13); its final track
histogram was blank. An offline sampled Sky Beta envelope check found up to
0.106 m penetration at the unchanged net collision geometry. This is a failed
simulated course attempt, not a successful traversal. The observer video and
raw pose/mission/ULog evidence are preserved in
`artifacts/gb10-sky-beta-astar-net-v1-*`. See
[SKY_BETA_SIM.md](SKY_BETA_SIM.md) for diagnosis and next steps.

Sky Beta now uses the actual CAD plus explicit user-authorized propulsion
estimates (1.2573 kg total). VIO and disarmed native Offboard checks pass.
The first armed hover attempt was stopped by native ESC failure and PX4
disarmed before takeoff; zero armed minimum had left two motors stopped.
The original log and actual observer video are preserved in
`artifacts/gb10-sky-beta-hover-v1/`. A fresh retry uses a documented spinning
armed idle and corrected hover mapping passed: **15.612497 simulated seconds
from native arm to disarm**, maximum estimated rise 0.4757 m and XY displacement
0.01922 m, with actual takeoff/landing detection and no ESC failure.
[Measurements](artifacts/gb10-sky-beta-hover-v2/native-measurements.json),
[mission result](artifacts/gb10-sky-beta-hover-v2/course-mission.json),
[parameter audit](artifacts/gb10-sky-beta-hover-v2/parameter-verification.json).
This time includes a small hover qualification, not the course; it must not be
compared with the airborne 7.04 s proxy path. The oblique camera hid the launch
lane; the overhead repeat passed in **15.599998 simulated seconds arm to
disarm**. Its full arm-to-disarm estimated XY displacement, including native
landing, was 0.03243 m; the 0.03 m guard applies to takeoff, not native Land.
[Actual hover recording](artifacts/gb10-sky-beta-hover-v3/video/flight.mp4),
[repeat measurements](artifacts/gb10-sky-beta-hover-v3/native-measurements.json).

The first course position-follower trial (`gb10-sky-beta-outbound-v1`) took off
but failed in the first corridor when PX4 withdrew heading validity. Actual
camera frames became nearly uniform gray, OpenVINS had zero visual updates,
and covariance exceeded the intact vision bridge bound. No landing/disarm was
established; the bounded simulator was stopped. Independent scene pose capture
and sampled geometry checks show no penetrating envelope contact before the
failure. Full physical parameters still match the reviewed native log.

The published net is represented by featureless sheets. An explicitly
estimated strand-rendering scenario has the **identical collision hash**, with
no obstacle or aircraft change. A separate sensor-only check at 1 m passed and
showed the strands in the actual camera image. Four position-follower retries
in that appearance scenario did not complete: v2 exceeded its tracking guard
at the first turn and did not establish landing/disarm; v3 landed after an
early straight-segment abort; v4 landed after an early curve abort; v5 crossed
two configured center-plane gates and landed after the guard tripped before the
third. These are native SITL outcomes, not completed-course scores.

The first A* stop/turn/go course run also failed. It launched, then the
independent scene stream shows contact with the north net at 127.225 s. The
tracking guard requested Land at 128.471 s; PX4 confirmed landed/disarmed by
133.671 s. Offline odometry comparison found 0.670 m position error while PX4
reported about 0.00035 m² position variance (approximately 1.9 cm standard
deviation). OpenVINS had zero features in 837 of 928 MSCKF updates. The
sampled Sky Beta envelope screen found up to 0.1056 m penetration. This is a
failed run with no course time or return; its observer recording and evidence
are in [the A* review](artifacts/gb10-sky-beta-astar-net-v1-review/README.md).
The first three disarmed VIO fixture runs are invalid as free-space accuracy
checks: the rail axis was rotated into the north net, and v2/v3 scene paths had
repeated aircraft-envelope contact. Their previously reported 0.0361 m and
0.0257 m errors do not characterize free-motion VIO accuracy. The feature-track
recordings remain usable as evidence from those contact-contaminated motions.
A corrected software-rendered disarmed run then recorded 5,181 VIO odometry
messages. Its course-forward fixture interval (sim 13.94?49.5 s) passed the
independent envelope screen with zero sampled contacts; offline position error
was 2.79 cm RMSE / 3.78 cm maximum over 3,052 samples. This short, slow motion
is not course-speed estimator qualification. The v6/v7 disarmed captures recorded actual SLAM PointCloud2 features. At the
1.0 m/s v7 rail speed, its contact-screened interval had 270 clouds, median 18
finite points, and a 1.82-second longest run below 8; the earlier 10/3-second
candidate would have faulted during a 3.17-second run below 10. The live route
guard now uses 8 features / 2.5 continuous seconds. The A* flight did not record
the PointCloud2 stream, so this is still provisional. Full evidence:
[v6](artifacts/gb10-sky-beta-vio-quality-v6/README.md),
[v7](artifacts/gb10-sky-beta-vio-quality-v7/README.md).

## Teammate PPO baseline adopted

Fresh original-code evaluation: nominal **7.04 s**; randomized **256/256**,
best/median/worst **6.84 / 7.08 / 7.30 s**. Three ONNX deployment-core frame
tests also reached hover in 7.04 s. Independent nominal checks found all nine
configured gate planes crossed in order, center height at most 1.153 m below
1.727 m walls, and zero tilted-envelope penetrating contact samples at 5 ms
against the decomposed original mesh. This uses an assumed 0.25 m radius.

[Fresh rollout video](artifacts/teammate-fresh-replay-v1/rollout.mp4),
[half speed](artifacts/teammate-fresh-replay-v1/rollout-half-speed.mp4),
[raw results](artifacts/teammate-reproduction-v2/result.json),
[architecture review](TEAMMATE_ARCHITECTURE_REVIEW.md).

The recording renders actual NumPy-policy states using MuJoCo, rather than
simulating the policy through MuJoCo/PX4 dynamics. Timing starts airborne and
ends at a pad hover; landing/disarm and actual hardware clearance remain pending.
Do not compare this 7.04 s to a full scored flight including landing.

## Current evidence, November 3 local / November 4 UTC

The initial path planner is four-connected A* on an observed occupancy slice,
followed by footprint-checked line-of-sight shortcuts. Unknown space is blocked.
It is not yet connected to a full live course mission. Hard corners require a
stop until collision-checked curved trajectories are implemented. The separate
speed component bounds acceleration, braking, curvature and sensing latency.

Measured laptop CPU component results, 30 repetitions after three warmups:

| Synthetic observed map | Reachable | A* + shortcut median / p95 |
|---|---|---|
| Straight corridor | 30/30 | 6.05 / 8.59 ms |
| Alternating maze | 30/30 | 40.38 / 53.41 ms |
| Unknown barrier | 0/30, correctly rejected | 4.36 / 5.30 ms |

Settings, maps, raw timings and source hashes:
[planner result](artifacts/planner-benchmark-v1/result.json). The 0.15 m radius
is a component benchmark setting, not a qualified X500 or Sky Beta footprint.

GB10 Gazebo/PX4 now receives real stereo/IMU OpenVINS estimates through the
external-vision bridge. A physical bootstrap fixture initializes the intact
disarmed aircraft, then releases it onto the floor. Actual QGroundControl is
connected. Calibrating the VIO world heading from fresh pre-fusion PX4 compass
attitude restored `heading_good_for_control`; no world pose is fed into VIO.
The latest read-only hover check still refuses flight because the current
AUTO_LOITER mode requires global position. A separate disarmed Offboard-mode
diagnostic is prepared to test native mode-dependent preflight checks.

Evidence: [latest runtime result](artifacts/gb10-gcs-preflight-v2/result.json),
[read-only hover refusal](artifacts/gb10-gcs-preflight-v2/hover-preflight.json).
The course-only sensor-rig [recording](artifacts/gb10-course-rig-v1/video/preview.mp4)
shows constrained sensor motion, not autonomous flight. Exact source timestamps
are in its recording manifest. Full course videos and measured finish times
remain pending. OpenFly weights are verified on the laptop; GB10 transfer and
isolated inference setup are in progress. See MODEL_BENCHMARKS.md.

The integrated project suite now passes **185 tests**, including the ROS mapping
adapter helpers, disarmed probe/harness and watchdog regression. This does not
establish live mapping or flight. Historical entries below preserve earlier
failures and describe the state at that time.

The generated [benchmark report](artifacts/benchmark-report-v3/report.md)
validates the hashes of preserved source/settings/evidence snapshots. It keeps
the failed readiness check distinct from the successful planner component test.
It also includes the failed first disarmed Offboard probe and its
[recording](artifacts/gb10-disarmed-offboard-v1/video/preview.mp4). PX4 actually
entered Offboard and reported native preflight true, but the adapter discarded
its native ACK because it misinterpreted `from_external`. The adapter now uses
the pinned Commander ACK semantics, with command/target/freshness and actual
state checks retained; a fresh bounded disarmed run is in progress. No flight.

Gazebo + PX4 is the primary end-to-end benchmark target; its flight integration
remains pending. MuJoCo is a working optional sandbox. A scene-only preview exists at
`artifacts/mujoco-preview-v1/course-preview.mp4`, with stills and ideal rendered
depth. This is a frozen-state camera orbit, not one of the flight recordings
requested for strategy comparisons. No finish time is assigned to it.

| Strategy | Implementation | Flight outcome | Measured finish | Recording |
|---|---|---|---|---|
| Classical | A*, shortcuts, speed and mapping components tested; real VIO/PX4 fusion observed; live mission integration pending | NOT RUN | -- | -- |
| Teammate PPO | Original code reused; 7.04 s nominal to hover, 256/256 randomized hover successes | NUMPY PROXY ONLY | -- | artifacts/teammate-fresh-replay-v1/rollout.mp4 |
| OpenFly-Agent | Released upstream weights; local inference adapter pending | NOT RUN | -- | -- |
| AeroVLA | Released upstream adapter; matching base and local inference adapter pending | NOT RUN | -- | -- |
| OpenFlyV2 | Upstream code/checkpoints marked coming soon | BLOCKED | -- | -- |

The MuJoCo scene has a generic quadrotor, actuators, cameras and inertial/range
sensors, but no estimator, flight controller or policy adapter. It is not a
ready-to-fly Sky Beta simulation. See PLAN.md for missing calibration, estimator,
flight qualification and recording work. This report will contain actual trial
results only after those prerequisites are established.

## Local verification, 2026-11-01

Before edits, `python -m unittest discover -s tests -v` failed with
`ImportError: Start directory is not importable: 'tests'`. There were no tests or
mission code in the downloaded folder. This is an unavailable initial test suite,
not a failing flight. No source or grader was changed to suppress that result.

After the new planning component: the same command passed 9 tests. Scene package,
synthetic profile and prepared OpenFly manifest CLI commands exited successfully.
After adding asset-preservation and scene-structure checks, see the final raw
verification log and source hashes in `artifacts/verification/`.

Tests cover acceleration/braking bounds, slowing before turns, sensing delay and
margin, unknown-space stopping, invalid inputs, infeasible initial speed,
analytical profile timing, route constraints, truthful manifest status, exact
asset preservation and static scene XML. XML parsing does not validate Gazebo
physics, rendering or a drone's ability to traverse the course.

Earlier local suite: **11 tests passed**, with raw output in
`artifacts/verification/unittest-final.txt` and code hashes in
`artifacts/verification/verification-final.json`.

A bounded 12-second static-world smoke check used the existing WSL Ubuntu 20.04
Gazebo 11.15.1. The initial invocation passed Windows CRLF into Bash, resulting
in an invalid world filename and fallback to the empty world. Its raw failure is
preserved in `artifacts/verification/gazebo-classic.txt`. Retrying with UTF-8 bytes
and LF input reached `Loading world file [...]course-classic.world` with no
reported file-not-found error; see `gazebo-classic-retry.txt`. Timeout exit 124
was the deliberate observation bound, not a mission pass. The graphics stack
printed `Dropped Escape call`; rendering, collision behavior and successful model
insertion remain unverified. No aircraft was present. Generated scene revision
is `artifacts/course-v3-r2`; the original export was preserved.

No team flight approval has been requested: an executable bounded mission does
not exist yet. The approval step comes after the mission is concrete and reviewable.

## MuJoCo validation

Executed with the project `.venv` using MuJoCo 3.16.0, NumPy 2.3.4, trimesh 5.1.1,
CoACD 1.0.14, Pillow 12.0.0 and imageio-ffmpeg 0.6.0. Original 11 tests passed
before edits; the new suite passed **15 tests**. Checks include open tunnel/window
centers, contacts with their shell/sill, four-motor gravity balance and sensor types
without a position fix. They are local geometry/dynamics unit checks, not flight.

Preview: 72 frames at 960x540 and playback 12 fps, recorded in ~3.37 wall seconds
(~21.4 rendered frames/s). Simulation time remained zero. See `preview.json`.
Passive physics smoke check: zero thrust for 2400 steps at 1/240 s; ~10 simulated
seconds in ~0.05 wall seconds, final quad center Z ~0.0349 m and 4 contacts.
See `physics-smoke.json`. It excludes camera rendering, VIO, PX4 and VLA inference;
do not extrapolate it into full-stack real-time performance.

## Local Gazebo/ROS integration work

Installed in Ubuntu-22.04 WSL: ROS Humble, Harmonic 8.15.0, OSRF's Humble/Harmonic
bridge packages and native build dependencies. PX4 v1.16.0 and matching messages
are compiled. This is an X500-based local proxy, not organizer Sky Beta validation.
Exact sources/packages and the PX4 binary hash are in artifacts/runtime/runtime-lock.json.

The proxy removes upstream NavSat from a copy without modifying upstream models;
there is no fake vision odometry source. A new read-only observer detects frozen
source timestamps separately from fresh callbacks. A conservative observed-grid
A* path component blocks unknown space and insufficient footprint clearance.
Unit checks currently pass **28 tests**; the initial suite of 15 passed before
these changes. Generated images preserve invalid depth values rather than filling them.

Bounded 30-wall-second sensor checks are saved in gazebo-sensor-smoke-v1/v2/v3.
The first check received 57 images per stereo eye, 19 RGB and 13 ideal-depth images.
Source timestamps show ~15 Hz stereo and ~240 Hz IMU/clock in simulation time;
wall time is slower. RGB/depth delivery loss remains unresolved. Source CameraInfo
reports 640x400, fx~381.36 and baseline 0.075 m. No TF/estimator accuracy follows
from that observation alone. Each test stopped its children and sent no commands.

Initial snapshots in v2 show an opaque-net/lighting import problem. The export now
separates visual groups, preserves MTL diffuse colors in SDF, and makes net visuals
transparent/non-shadowing without removing net collision. Intermediate snapshots
and all earlier scene assets remain. The next proxy initializes yaw along the
first corridor (+Y) rather than toward the neighboring blank wall; this is a
configurable scene starting condition, not a navigation algorithm.

The first PX4 build attempt failed because the Windows log helper tried printing
Unicode progress bars through cp1252. The preserved log is build-px4-01.log.
After correcting UTF-8 output and disabling pip progress bars, PX4 built successfully
in build-px4-02.log. No estimator checks, PX4 safety settings or graders were weakened.

### Subsequent local checks

The later sensor smoke test `artifacts/gazebo-sensor-smoke-v8/` received about
109–110 frames per image stream. Matching the bridge's reliable image QoS
resolved the loss observed in the earlier checks. STL visual meshes with explicit
facet normals also restored the corridor appearance; the original OBJ collision
mesh remains in use. Earlier results and diagnostic images remain available.

Disarmed PX4 checks v3–v5 received telemetry but reported invalid horizontal
position and failed preflight checks, including magnetic interference. These are
not flight passes. Shared simulation-clock configuration resolved the earlier
DDS synchronization failure. No external vision was supplied and no arming or
mission was attempted. The current unit suite passes 30 tests.

OpenVINS compiled successfully: all three packages finished in 20m54s; see
`artifacts/runtime/build-vio-01.log` (exit code 0). A calibration derived from
the proxy camera and IMU is saved in `artifacts/vio-calibration-v1/`.
This calibration is for the virtual sensors only. Estimator output,
accuracy, real-time performance and PX4 integration remain unverified. There are
no autonomous course times or recordings yet.

### Parallel component implementation

The combined suite now passes **49 tests**, including seven calibrated depth-map
cases, ordered mission-event tests and five recording tests. The latter include
an actual FFmpeg encode/decode check. These are component checks, not course runs.
Mapping is conservative with sticky occupied evidence and no dynamic clearing;
mission inputs still require real detectors and estimator-backed clearance;
recording currently accepts offline PNG sequences rather than a live ROS stream.

The first shadow VIO run (`artifacts/vio-shadow-v1/`) failed during startup with
a segmentation fault, without any odometry output. The debugger log
`artifacts/runtime/vio-backtrace-01.log` identifies an image-transport publisher
call in a worker thread. A second isolated run disables upstream publisher
threading via its standard option; its logs also show a startup failure. Neither
run starts PX4 or supplies external-vision data. All failures are retained.

### GB10 preparation

SSH access succeeded to the team's separately authorized ARM64 Ubuntu 24.04
GB10. The project/course assets and pinned PX4/model sources were transferred
into user-owned directories. Native component tests: **44 passed, 5 skipped**
in 0.74s; skipped checks require optional MuJoCo assets/runtime or the FFmpeg
Python package. This does not replace the laptop's full 49-test result.
See `artifacts/runtime/gb10-user-prepare-01.log`.

Gazebo/ROS are not installed yet: sudo requires interactive authentication and
the account cannot access Docker. The reviewed admin installer and sensor-only
test are prepared in `sim/runtime/`; see `sim/runtime/GB10.md`. No GB10 simulator,
autopilot or flight has been run, and existing GPU workloads were left alone.

### GB10 Gazebo sensor qualification

After the team installed dependencies, the native Jazzy/Harmonic 8.15.0 sensor
test passed. `artifacts/gb10-sensor-smoke-v1/` contains the result, observation,
logs and snapshots. In 30 wall seconds: left stereo 269 images, right/RGB/ideal
depth 268 each; clock and IMU timestamps advanced without missing image streams.
The observed clock span was about 18.06 simulated seconds (roughly 0.60 overall
simulation/wall ratio including discovery). Other GPU work was active; this is
not an isolated performance benchmark. All child processes stopped cleanly.

The separate renderer log confirms `GL_RENDERER = NVIDIA GB10/PCIe` and NVIDIA
driver 580.173.02 despite earlier device permission warnings; see
`artifacts/runtime/gb10-rendering-01.log`. No PX4, arming or flight was involved.
OpenVINS native ARM64/Jazzy compilation is underway; positioning is not validated.

The second GB10 sensor test also exercised live RGB capture and video encoding:
271 PNG frames with source timestamps, spanning 17.820830482 simulated seconds.
The returned MP4 decoded to exactly 271 frames. Local viewing copy:
`artifacts/gb10-recording-preview-v1/preview.mp4`; source PNG evidence remains on
the GB10 under `artifacts/gb10-recording-smoke-v1/rgb-sequence/`. This is a
stationary camera/aircraft sensor recording, not an autonomous flight or finish
time. The eight focused recording/snapshot tests passed before the run.

The initial native OpenVINS build later failed in `ov_msckf`: Jazzy removed the
old `image_transport.h` and `tf2_geometry_msgs.h` compatibility headers. Core and
initialization packages built. Logs `gb10-build-vio-01.log` and `-02.log` preserve
the failures. A minimal `.hpp` include migration is recorded by
`sim/runtime/rebuild_gb10_vio_jazzy.sh`; rebuild `-03` is in progress. No estimator
or flight is running. These changes do not change estimation checks or PX4.

### Foreground VIO motion diagnosis

The watchdog completed the native build and corrected a detached worker capturing
a stack-local IMU measurement. Foreground review then found the executable forced
subscriber threading on after parsing options. Removing that override permits
the existing synchronous mode; no estimator readiness conditions were changed.
The static course still provides no full VIO odometry readiness while initial
zero-velocity updates continue. Shutdown initially remained unhealthy.

`sim/sensor_rig.py` creates a separate constrained prismatic camera/IMU rig. It
retains actual rendered stereo and physics IMU, with bounded targets +0.12 m,
-0.12 m and 0 m; no PX4 or flight commands. A checkerboard exists only in this
diagnostic scene. It does not qualify feature-poor competition geometry.

Rig v1 lacked useful features. Rig v2 initialized but the observer crashed on
OpenVINS 16UC1 depth. Added stride/endian-correct raw decoding, preserving units
and zeros; the new focused test passes. Its initial test-file collection error
was corrected before the final four snapshot tests passed. All failures remain
in command/run evidence.

Rig v3 produced **15,128 odometry samples**, 15,127 source advances, no timestamp
regressions over 63.229 simulated seconds. The final pose/covariances were finite
and quaternion norm 1.0; this checks only the last sample, not trajectory accuracy.
OpenVINS still aborted at shutdown (250), despite successful sensor delivery.
Evidence/video: `artifacts/gb10-vio-rig-v3/`; full source frames remain on GB10.
The harness now treats abnormal OpenVINS shutdown as failure explicitly.

Explicit ROS object teardown passed in rig v4: 15,127 odometry messages, 15,126
source advances, no regressions, over 63.11249 simulated seconds. OpenVINS,
Gazebo, bridge and observer all exited 0 and all children stopped. Native patch
is saved under `artifacts/vio-review-v1/native-with-shutdown.patch`; result is
`artifacts/gb10-vio-rig-v4/result.json`. This is a functioning sensor-rig VIO
pipeline, not demonstrated accuracy or readiness on the actual course. The combined local
suite passes **54 tests**. No flight, course completion time or hardware
qualification follows from these diagnostic tests.
