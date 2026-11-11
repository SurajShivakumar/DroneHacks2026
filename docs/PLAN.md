# DroneHacks 2026 autonomy and benchmark plan

## Status — November 6 (evening): sub-180 s completion (v51, 146.42 s)

v51 completed takeoff → 9/9 ordered gates → pad landing/disarm in 146.42 s arm→disarm; truth evaluation: zero structure contacts, 3.6 cm from pad centre, tilt-aware gates 9/9 (min lateral margin 10.4 cm). v53 repeat (gentler profile) 150.27 s, same checks. Pipeline: published-map robust-corner path (10 cm smoothed, re-audited) → heading DP on published-geometry view score (turns while moving) → speed profile (0.45 m/s, 0.12 m/s², 0.25 rad/s, stop at 14 path kinks) → PX4 setpoints with new acceleration/yaw-rate feedforward. Failures preserved: v41 (v40 repeat, VIO loss turning at tube mouth), v42/v43 (kink overshoot), v45–v49 (tube-mouth stop overshoot without feedforward), v48 (stale VIO pre-arm with 4 parallel sims), v52 (VIO dropout). Details: `artifacts/gb10-sky-beta-course-v51/README.md`. Parallel runs: `sim/runtime/gb10_slot_run.sh` (max 3). Next: success-rate repeats, cuVSLAM (organizer estimator), online stereo mapping, laptop WSL2 mirror qualification.

## Status — November 6: first complete simulated outbound run (v40)

Run v40 completed takeoff → all 9 ordered route gates → pad landing/disarm in
498.45 simulated seconds (arm→disarm), native OpenVINS/PX4, estimated Sky Beta.
Independent truth evaluation: zero course-structure contacts, whole envelope
inside the pad ring (10.8 cm off centre), tilt-aware gate audit 9/9 (strict
auditor's full-radius tilt fallback flags corner1; preserved and explained).
VIO drift 6.9 cm RMS / 15.4 cm max. Recording and full details:
`artifacts/gb10-sky-beta-first-completion-v40/README.md`. n = 1; not a valid
timed finish (180 s limit). It relies on an estimated textured-wall scene.

Root causes found and fixed on the way (v34–v39): fixed north yaw stared at the
net (one-cell stereo alias) → offline perception-aware headings fitted to
recorded feature counts (`tools/prepare_heading_reference.py`); flat untextured
walls give VIO nothing → estimated wall texture scene; EV-bridge/recorder 600 s
caps; two clock races in the feature watchdog; turn-stop overshoot from the
tracking clock's progress floor → slowed approach into turns; dark tube wall →
tube-axis heading. Next: repeat runs for a success rate, test on the flat-colour
scene with online heading choice, then speed (turns and approaches dominate).

## Previous priority: one complete flight

Suraj narrowed the immediate goal to one proper autonomous takeoff, every
checkpoint in order below the walls, and pad landing/disarm, regardless of
speed. Speed optimization, return bonus, RL and VLA comparisons are deferred
until that outbound flight is independently verified. Use native OpenVINS/PX4;
the failed experimental shadow filter remains disconnected. Prepared settings
are `configs/sky_beta_first_completion.json`: the cleared route slowed to
134.94 s reference time, peak 0.267 m/s, stopping at every vertex. Longer
supervisor duration bounds permit a completion diagnostic; they do not change
the official 180 s limit or its independent reporting. All sensor, authority,
reset, deviation and preflight checks remain intact.

After server recovery, GB10 had no live Gazebo/PX4 processes. The interrupted
background-appearance generator was completed/tested locally. A disarmed
0.9 m translation check at 0.25 m/s is now running as v28, with native VIO and
unchanged guards. This separately labeled estimated exterior tests missing
venue context outside the original net; it changes no course collisions and
is not evidence of real venue appearance. Its result is pending.

## Remaining work to completion — November 5

Surface-appearance v27 also failed the unchanged native feature guard (626
callbacks, never READY), despite offline VIO drift of 8.00 mm RMS / 26.43 mm
max in its fixture interval. Its camera still mostly sees repeated net grid
against a blank exterior. Video and evidence are in
`artifacts/gb10-sky-beta-vio-surface-v27/`. New offline work separates an
estimated arena-background scenario and a native-resolution stereo variant;
neither is yet runtime-qualified. Latest full suite: 340 tests / 52 subtests.

Latest full-turn check: v6 processed 1,292 live frames with a commanded
90-degree yaw. Independent source 15..75 s evaluation found raw VIO XY
6.55 mm RMS / 30.16 mm max, but the added filter reached 103.30 mm max while
braking. It remained at about 0.52 m/s after the physical fixture stopped.
The constant-velocity model / rejected innovations need correction; the shadow
filter must not feed PX4. Actual sources, failure transient, evaluation and
video are in `artifacts/live-stereo-shadow-v6/`. All simulator children stopped.
The supplied materials have flat colors without texture maps/UVs; a separate
estimated surface-appearance sensitivity scenario is being prepared, retaining
the original geometry/collisions and all original failed evidence.

New live-shadow milestone: `artifacts/live-stereo-shadow-v5/` contains1,302
processed live frames and an observer video. Actual stereo/VIO/range data with
declared-heading calibration ran at25ms median on GB10. Independent source15..75s
evaluation found filtered XY9.54mm RMS/46.23mm max versus raw VIO8.64mm/24.96mm:
the filter did not improve that easy view and is not airborne-qualified. The
earlier fixed-yaw v4 failed0.804m RMS because VIO yaw axes changed across runs;
its evidence remains intact. The full-turn result above supersedes this view.
Native feature guards and PX4 odometry inputs remain unchanged. Full local
verification now passes334 tests and52 subtests.

Parallel offline work prepared `artifacts/published-course-efficient-v1/`:
the same17.868m public-map A* path retimed to89.9626s with0.4m/s cruise,
bounded acceleration/jerk, braking at every vertex and no stationary yaw spins.
The strict existing mesh screen passes with8cm extra margin; all9 diagnostic
ordered route planes pass. Fixed north yaw is an unqualified perception choice.
The initial 30 s launch/landing and 40 s return scheduling example is not
feasible under a fresh return qualification. The separately checked
`artifacts/published-return-over-barrier-v2/` reference takes 27.05 s airborne,
at 2.10 m transit height, with zero mesh penetrations at 8 cm margin. Current
30 s idle-health plus 30 s Offboard-preflight policy adds 60 s before actual
return launch/landing: at least 87.05 s for that phase. A faster outbound or a
separately reviewed continuity/qualification policy is necessary for the
180 s return bonus; safety checks have not been reduced. Actual complete-flight
timing is unmeasured. Do not call either reference a course finish.

Latest qualification update: ground-height fixture results did not transfer
to flight-height imagery. The bounded powered corner attempt launched, then
the feature watchdog requested Land on the first straight; PX4 confirmed
landing/disarm (ABORT_LANDED). Its verified observer clip is
`artifacts/gb10-sky-beta-corner-qualification-v1/flight-only-video/preview.mp4`.
It did not reach its endpoint or complete a checkpoint/course.

At 0.9 m, disarmed stereo VIO reached 27 cm maximum error; at 0.6 m it reached
16.5 cm. A monocular diagnostic also failed (1.44 m maximum). Feature-budget
increases helped lower-height quarter-turn tests but did not qualify actual
flight-height views. Repeated net-grid correspondences remain a perception
problem; no readiness/authority/estimator thresholds were weakened.

Actual recorded stereo depth also exposed periodic aliases surviving ordinary
left/right and photometric checks. Independent ambiguity rejection removes
many returns but still leaves bad phases. Offline depth-to-published-map
components, scalar wall-normal observations and published-geometry raycasts
now have tests; they have not been connected to flight. Ideal rendered depth
is restricted to disarmed diagnosis and independent evaluation. Current work
tests actual stereo with explicitly declared per-pixel published-map priors.
The sparse recursive eight-state position/velocity/VIO-bias experiment reached
8.82 mm XY RMS and 16.96 mm maximum over 29 recorded small-angle fixture frames,
with a 44.3 ms median sensor computation. Only 6 frames constrained both XY
directions; unobserved uncertainty continues to grow. The velocity-only
predecessor failed (12.1 cm RMS) and is preserved. These results use actual
stereo; independent scene poses enter only after estimation for evaluation.
Evidence: `artifacts/stereo-kinematic-active-view-v2/`. This is offline,
fixed-height, local-motion evidence, not course completion or flight readiness.

`tools/observe_published_stereo.py` now provides a live read-only shadow against
ordinary stereo, CameraInfo, VIO and downward LaserScan topics. It checks causal
support, frames/calibration, source and arrival age, projected range height,
and processing time; it publishes no topics and does not change PX4 input.
Its static-init alignment is an explicit diagnostic configuration, not flight
startup calibration. The first launch used the wrong ROS domain and lost its
SSH session before cleanup: v21 evidence is preserved and its disarmed orphan
groups were stopped. The v22 retry uses domain 42 and a detached bounded driver.
Live timing/accuracy qualification remains pending. The full local suite passed
325 tests and 52 subtests after adding causal-source/range/image contracts.

Current milestone: the CAD-based estimated Sky Beta has flown bounded hover /
landing trials. Full-course completion remains unproven. The v10 disarmed
feature observer reached READY and recorded 361 callbacks with no quality fault.
The v9 floating-point timestamp fault is preserved; regression checks now cover
roundoff, genuinely future timestamps, stale arrivals/sources and track loss.
Historical sections below describe earlier states; this checklist takes priority.

1. **Qualify estimation and navigation.** Preserve v10 evidence/video and pin
   all watchdog source in future run snapshots. Measure drift and covariance
   through turns, the tunnel and low-texture views using separate offline
   scene evidence. Validate feature-loss/stale-data abort behavior. Finish
   live depth/TF mapping, estimator-reset handling and height-aware clearance
   if using sensor-driven A*. Keep ground truth outside the controller.
2. **Prepare and complete one conservative outbound baseline.** Audit the
   entire Sky Beta envelope against ordered checkpoints, opening heights and
   the below-wall requirement; qualify the independent evaluator including
   contact penalties and actual landing/disarm. Record the concrete mission
   settings and flight approval required by AGENTS.md before arming. Use a
   slow, acceleration-limited trajectory with turn braking, then launch,
   traverse all checkpoints, land and observe disarm. Preserve failed trials.
   Require observer video, native telemetry/ULog and independent checkpoint /
   clearance evidence before reporting a course finish time.
3. **Qualify the return bonus.** After outbound success, check a return route
   over the barrier, explicitly allowed by supplied rule 7.10. Check actual
   aircraft/net/barrier clearance, gated rearm, takeoff, home landing and
   disarm within 2 ft of the launch mark. Outbound and return together must
   fit the supplied 180-second limit. Reverse-course navigation is optional,
   not a bonus requirement. Verify bonus eligibility separately from outbound.
4. **Complete the strategy comparison.** Run the classical trajectory/A*
   baseline and the teammate PPO through the same estimator, mission and
   evaluator contracts. Qualify model-action adapters before closed-loop VLA
   flight. OpenFly BF16 inference works on GB10 but requires a measured
   memory-reduced variant for the 8 GB Jetson. AeroVLA needs downward RGB
   absent from Sky Beta; label added-sensor experiments accordingly. The third
   candidate had no released checkpoint in the saved review: recheck release
   availability or select a runnable replacement. Do not promise three valid
   hardware-compatible VLA results until those dependencies are resolved.
5. **Improve speed after reliable completion.** Benchmark repeated seeds and
   estimated mass/thrust/latency variants. Optimize straights, acceleration,
   braking and turn curvature; compare success rate, clearance, contacts and
   complete mission time. Add residual RL / Isaac Lab training only against
   the validated baseline, then retest through Gazebo/PX4. Convert chat route
   preferences to saved, collision-checked settings before each experiment.
6. **Finish requested benchmarks and actual competition submission.** Keep
   code/model/settings, successes and failures, checkpoint/end-to-end times,
   and sim recordings for the comparisons Suraj requested. A separate delivery
   package is not a named challenge requirement. The supplied rules require
   the team repository link by Sunday 12:30 PM and all flown code pushed by
   4 PM (9.3), a README explaining the stack and how to run it (9.4), credits
   for reused dependencies (9.1), and a 5–7 minute technical presentation.
   These times are from the supplied rules, not a newly verified event schedule.
   Hardware transfer separately needs the actual Jetson stack pinned, onboard
   memory/latency measured, sensors calibrated and estimated dynamics checked.

Immediate work: resolve recorded stereo/map-observation ambiguity at actual
flight height, validate an estimator correction independently, then repeat
the bounded corner qualification. Feature gate, yaw fixture, video capture and
safe quality-triggered native landing are exercised; reliable navigation is
not yet established. Simulator and real-aircraft qualification remain separate.

The current published-map stop/turn/go reference is 232.97 seconds before
takeoff/landing, so it cannot be a valid timed completion under rule 7.9 even
with perfect tracking. It remains a diagnostic reference. A new competition
reference needs a measured motion/turn budget that reserves time for launch,
landing and any return; speed must not be increased solely to fit the limit
before estimator/clearance qualification. The supplied rules also require
the whole drone to cross actual checkpoint lines, not merely the center.

## Objective and current status

Sky Beta update: actual STEP/STL geometry, component masses, COM and full inertia
replace the X500 proxy. Suraj authorized estimating unavailable dynamics.
The active estimated model includes props, wiring and rangefinder, totaling
1.2573 kg, with explicit motor assumptions in `configs/sky_beta_estimates.json`.
Stereo/IMU VIO, disarmed native Offboard preflight and the mission adapter's
read-only health checks pass. First armed attempt exposed a zero armed-idle
modeling error: two stopped rotors correctly triggered native ESC failure and
PX4 disarmed before takeoff. The v4 estimated model restores spinning armed
idle with recalculated hover mapping, retaining native safety checks. A fresh
0.5 m rise / hold / land / disarm retry passed: 15.6125 simulated seconds from
arm to disarm, actual native takeoff/landing detection, no ESC failure. Maximum
estimated XY displacement was 1.92 cm. The repeat with a clear overhead recording
also passed in 15.6000 s. This is a bounded hover, not a full course run.
The first course trial failed in the first corridor after feature-poor camera
images caused VIO uncertainty and native heading validity loss. All checks were
preserved. The estimated net-strand appearance variant has identical course
collisions and passed a sensor-only check. Its first A* stop/turn/go trial also
failed: independent scene replay found net contact and about 0.67 m position
estimate drift before the native tracking guard triggered a controlled Land.
No complete course finish is claimed; VIO quality in this scene must be
understood before another course attempt. See SKY_BETA_SIM.md.
The supervisor and teammate-policy/trajectory adapter implement navigation and
a gated return sequence; outbound qualification must precede return execution.

Priority update: adopt the teammate's existing PPO controller/training code and
trained policy from pinned `jakefromstatefarm00/dronehacks2026`. Fresh nominal
rollout reproduces 7.04 s airborne-start-to-pad-hover; randomized 256/256,
median 7.08 s. No wall hopping in the nominal trajectory; independent gate and
mesh-decomposition checks plus video are in TEAMMATE_ARCHITECTURE_REVIEW.md.
This excludes takeoff/landing/disarm and is not a Gazebo/PX4 finish. Reuse their
implementation rather than rebuilding it. Next: adapt their core to the actual
estimator/ROS/PX4 safety contract and qualify flight/landing. A* remains useful
for changed layouts; VLA work is secondary to a reliable small-policy baseline.
Consider Isaac Lab after this baseline transfers; keep the same contracts and
retain Gazebo/PX4 validation.

Latest update (November 4): initial navigation uses
observed-grid A* with conservative footprint inflation, then checked shortcuts
and a separate acceleration/braking speed profile. Synthetic CPU measurements
and limits are in BENCHMARKS.md. Live mapping-to-controller integration and
full height-aware course navigation remain unfinished. The new ROS depth/TF
adapter publishes a single-frame observed occupancy grid, with explicit units,
calibration and source freshness; it still needs live estimator-TF validation
and pose-quality/reset handling before flight use. GB10 now runs native
Gazebo/PX4/Jazzy, actual OpenVINS external-vision fusion and QGroundControl;
disarmed aircraft bootstrap and heading alignment have been observed. A
mode-dependent preflight diagnostic has passed. OpenFly BF16 inference now has
a complete verified GB10 benchmark record: 0.741909 s median, no flight control.
The A* full-mission attempt is preserved under
`artifacts/gb10-sky-beta-astar-net-v1-*`; its observer recording is a failed
diagnostic run, not a finish benchmark. Return remains disabled pending an
independently verified outbound finish. Later historical sections are not
current status.

Build a sensor-driven autonomous finish, benchmark it without task fine-tuning
against three aerial VLA candidates, preserve recordings and measured times, then
improve speed and optional bonuses. Allow route experiments requested in chat,
compiled to saved pre-run settings. Never alter an active scored run except abort.

As of 2026-11-01, the local project originally contained four instruction/access
files and the three course OBJ/MTL/README files. The supplied competition rules
and Sky Beta specifications were also read. The companion mission, notebooks,
estimator, calibration and tests mentioned in AI_CONTEXT.md are not downloaded.
No connected simulator document session was found. The user clarified that the
downloaded geometry should be used to build a local simulator.

Implemented now: offline path-speed component, route preference validation,
strategy registry, preparation manifests, synthetic example, tests, a MuJoCo
course/proxy quadrotor with RGB/depth rendering and IMU/range sensor definitions,
a scene-preview recorder, and an earlier static Gazebo course packager.
Gazebo + PX4 is the primary target for end-to-end flight benchmarks, following
the organizer's pinned starter stack. MuJoCo remains a working lightweight
preview/physics sandbox; Isaac Sim is not required. Gazebo flight integration
is pending, so this target choice is not a claim of a working flight benchmark.
The local Gazebo/ROS dependency stack is now installed in Ubuntu-22.04 WSL.
PX4 v1.16.0 and matching messages have built; raw stereo/RGB/depth/IMU/clock
streams have been observed over ROS for bounded 30-second sensor checks.
Read-only telemetry diagnostics and a conservative A* occupancy-slice component
are implemented. The proxy source/package/binary lock is under artifacts/runtime/.
Not implemented: live mapping/perception, VIO, full height-aware collision-checked
trajectory generation, ROS/PX4 mission execution, model inference adapters or evaluator.
No flight, measured finish or VLA inference exists yet. A clearly marked static
MuJoCo preview video is saved under `artifacts/mujoco-preview-v1/`.

## Competition priorities and rules

The supplied rules award 60 run points and 40 judged points (25 presentation,
15 generality). Ordered checkpoints are corridor 4, maze exit 7, tunnel 10,
chicane 7, landing 7. A fully below-wall finish scores 35 + 20*T_fastest/T_finish,
plus 5 for the window. Successful return multiplies run points by 1.25. Unfinished
runs get their cleared checkpoint points plus up to 7 speed points. Contact adds
5 seconds to that and later checkpoint times. Official scoring remains external.

Prioritize completion, then smooth speed, return and window qualification. A return
failure loses the bonus rather than the completed outbound run under supplied rule
7.10. Verify current event amendments before relying on this scoring description.
All course checkpoints must be crossed by the entire aircraft in order.

The net is 6.096 x 6.096 m and approximately 3.048 m tall. Most walls are
1.7272 m high. The landing outer ring is approximately 1.524 m in diameter.
OBJ units are metres, Z up, origin at floor center. Mesh object names and bounds
are a published prior, not authoritative clearance: inspect actual openings.
The layout is preliminary and may change.

One external start command, then abort only. All competition inference/planning
must run onboard. Read-only remote monitoring is permitted. No GNSS, external
positioning, altered course or copied practice-run maps. Published maps are
allowed; hard-coded layouts affect generality. Ask organizers how rule 6.4 applies
before training weights directly on recorded physical course runs. Credit models
and libraries and respect the event's build/submission requirements.

## Hardware and software contract

Sky Beta: Jetson Orin Nano 8GB, Pixhawk 6C/PX4, OAK-D S2 stereo/RGB camera and
downward VL53L1X. Team-owned local context says lab ROS 2 Jazzy, while supplied
hardware description says Humble. Pin the real deployment's JetPack/CUDA/ROS/PX4,
camera driver and `px4_msgs`; do not assume interchangeable binaries.

Retain native ROS 2/`px4_msgs` over the reviewed XRCE-DDS connection. MAVROS is an
optional MAVLink alternative or source of conversion helpers, requiring an actual
supported MAVLink connection. Use one command owner and one reviewed coordinate
conversion boundary. ROS ENU/FLU, PX4 NED/FRD and optical-camera frames differ.
Transform poses, velocities, yaw and covariance consistently through calibrated TF.
Do not create a second agent in the managed lab or copy simulator PX4 parameters
onto hardware. Keep simulator state, observer views and reset APIs out of flight code.

## Architecture

1. Calibrated, timestamped stereo + IMU -> validated visual-inertial odometry.
2. External-vision bridge -> PX4 estimator, with health, covariance and reset checks.
3. Depth + floor-line/opening/pad perception -> local height-aware obstacle map.
4. Ordered mission state -> observed local goals and route preference.
5. Collision-checked path -> acceleration/braking speed profile -> trajectory.
6. Authority/freshness/constraint gate -> native PX4 setpoints.
7. Independent logs/evaluator/video -> evidence report.

Keep continuous control odometry separate from map corrections; a loop closure or
relocalization must not jump the trajectory reference. Inspect the organizer's
cuVSLAM wrapper, external-vision bridge and actual message definitions before
integrating. Topic discovery alone is insufficient: inspect at least 30 seconds
of actual timestamped streams. Use `/clock` and `use_sim_time` in simulation;
system clock on hardware. Observe actual delivery and real-time factor.

## Perception and stage behaviors

Use a local voxel map or several height layers; a flat occupancy map cannot model
the window, tunnel and return. Keep occupied, observed free and unknown distinct.
Invalid depth is unknown. Inflate obstacles for the aircraft's swept envelope,
tracking error and localization uncertainty. Account for tilt and propeller span.

Start with floor-line segmentation/curve fitting, depth-plane/opening geometry,
and pad shape/color tracking. Add learned detectors only when measured failures
justify them. Verify floor visibility at candidate heights and attitudes: a forward
camera can lose both the nearby line and the pad during descent. Remounting needs
inspection and fresh calibration.

Mission: PRECHECK -> TAKEOFF -> FIRST_CORRIDOR -> MAZE_EXIT -> TUNNEL -> CHICANE
-> LANDING -> DISARMED. Optional autonomous RETURN_PRECHECK -> TAKEOFF ->
CROSS_BARRIER -> RETURN_LAND -> DISARMED. Each state needs observation-based
entry/completion, confidence, timeout, full-aircraft crossing and failure behavior.

Maze: line-guided corridor following with depth clearance; observe the next corridor
before committing. Tunnel: align yaw/altitude/position before entry, evaluate stereo
tracking in darkness, never extrapolate indefinitely with stale odometry. Downward
range does not provide tunnel lateral position. Gate B is the initial finish route.
Window needs measured aperture and attitude-dependent clearance. Landing needs
target acquisition, usable final-descent sensing and verified landed/disarmed state.
Return may retain current-run state but must satisfy full clearance below the roof.

## Speed planning and learning

Accelerate on verified clear straights and brake before turns from the first
baseline. Separate geometric path from timing. Limit corner speed by
v <= sqrt(a_lateral*R). Limit speed by stopping visibility:
v*latency + v^2/(2*a_brake) + margin <= observed_free_distance.
Backward propagation enforces upcoming turn/end bounds; forward propagation
enforces acceleration. Rounded turns may carry speed; sharp discontinuities need
slowing or path smoothing. Current `dronebench/speed.py` implements these scalar
bounds. It does NOT implement collision checking, jerk constraints, coupled thrust
limits, a dynamics simulator, tracking or certification. Its default numbers are
synthetic starting examples, not qualified flight settings. Endpoint infeasibility
raises an error; ideal traversal time must never be reported as a flight result.

Measure tracking and braking first. Optimize lookahead, margins and speed with
small parameter searches before RL. Then try a small policy selecting a bounded
speed multiplier or trajectory residual from clearance, curvature, distance to
turn, estimated velocity, tracking error, estimator confidence and sensor age.
Keep checkpoint logic and authority checks outside the learned policy. Revalidate
every proposed trajectory; bounded output is not a safety guarantee.

RL curriculum: straight corridor -> turn -> tunnel -> chicane -> landing -> full
course, with held-out layouts and realistic sensing/dynamics variation. Measure
training throughput before choosing PPO/SAC infrastructure. A separate permitted
training simulator is not the managed lab reset interface. No privileged truth
inputs to deployed policies or synthetic replacement for a valid onboard fix.

## Three VLA candidates and comparison boundaries

1. **OpenFly-Agent**: released aerial navigation checkpoint
   `IPEC-COMMUNITY/openfly-agent-7b`, based on OpenVLA; image/history/instruction
   input. Native adapter, observation history and action normalization pending.
2. **AeroVLA** (formerly AerialVLA): released LoRA weights at
   `XuPeng23/AerialVLA` with matching OpenVLA-7B base; dual-view perception and
   continuous navigation/landing outputs. Native adapter pending; physical
   deployment remains listed as future work. Do not infer a second onboard view.
3. **OpenFlyV2**: report of 10 Hz onboard action chunks on Orin NX, not our Nano.
   Upstream repository currently marks code/checkpoints coming soon. BLOCKED,
   not an implemented third baseline. Do not replace it without identifying the
   substitution. Agile Autonomy is an optional non-language flight-policy comparator;
   AirVLA is an additional research candidate with unreleased code/data.

"Without fine-tuning" means using released aerial weights (including the author's
released adapter), with no new weight updates from our course. Record exact base,
adapter, revision, tokenizer, preprocessing, quantization, action normalization,
prompt and camera/history requirements. A manifest alone is not inference.

First run offline inference on simulator camera recordings, then a proposed-action
shadow test, then bounded closed-loop qualification. Keep native-policy evaluation
separate from hybrid goal-proposal-plus-classical-planner evaluation. Report the
common planner's role and intervention rate; do not attribute its success to a VLA.
Never fabricate inputs or action semantics to make incompatible checkpoints fit.

Estimate memory and benchmark inference with perception active. A 7B backbone at
FP16 uses about 14 GB before runtime/vision overhead, beyond an 8 GB Jetson;
quantization and latency still need validation. Competition inference cannot be
offloaded to laptop/cloud. Label desktop proxy results separately.

## Local simulator construction

Use Gazebo + PX4 for end-to-end mission qualification and reported baseline
course times. The supplied starter description specifies Gazebo, and PX4
documents a maintained Gazebo SITL integration with ROS 2 support. Match the
organizer's actual PX4/Gazebo/ROS versions before event-matched deployment: the downloaded
notes alone do not establish the release combination. Our Harmonic world is a
prepared asset, not evidence that the event uses that exact release. The locally
installed Gazebo Classic 11 is not the default target for new integration.
See https://docs.px4.io/main/en/simulation/ and
https://docs.px4.io/main/en/sim_gazebo_gz/ .

OBJ/MTL geometry is simulator-neutral. Choose the flight stack for its supported
aircraft, sensor plugins, estimator/transport and transfer path, rather than the
mesh format. Start with headless physics, low-resolution required camera streams
and optional observer recording; measure actual throughput before increasing
rendering load. Never improve throughput by bypassing estimator/sensor checks.
No new MuJoCo flight controller/bridge is planned for the initial competition
baseline; retain the existing MuJoCo tools for isolated geometry/perception/RL
experiments and label those results separately from PX4 benchmarks.

Use MuJoCo on Windows for lightweight local dynamics, RGB/ideal-depth rendering,
recordings and eventual policy experiments. Native rendering succeeded on this
laptop. `python -m sim.mujoco_course` imports the 33 OBJ groups and MTL diffuse
colors; CoACD decomposes concave collision meshes with a requested 1 cm metric
threshold and fixed seed. Do not use a convex hull for the whole course or tunnel:
it would close openings. Rendering uses original group meshes, with transparent
visuals for the net; net collisions are still present as solid-sheet proxies.

The generic quadrotor is an explicitly uncalibrated 0.8 kg rigid body with four
site-force/torque actuators, illustrative inertia and rotor envelopes, three
forward camera definitions (75 mm stereo baseline), gyro/accelerometer and downward
rangefinder. Its RGB/depth are ideal rendered outputs, not a measured OAK stereo
pipeline. Camera/range geometry and rates still need the organizer's calibration.
No ground-truth pose sensor, fake VIO, estimator fix, PX4 controller or native ROS
transport is provided. Model state is used only by scene tools and tests.

Observed: 15 tests passed, including open tunnel/window and solid shell/sill
contact probes plus thrust/gravity balance. A six-second 960x540 frozen-state
preview rendered 72 frames in approximately 3.37 wall seconds (~21.4 frames/s).
A separate 2400-step zero-thrust passive settling check advanced 10 simulation
seconds in ~0.05 wall seconds with no controller, rendering or perception. These
are smoke-check measurements, not closed-loop throughput or aircraft performance.
Preserve input hashes and validate the full swept-volume clearance separately.
Current geometry/lighting do not qualify realistic darkness, noise, optical
properties, downwash, motor dynamics, battery behavior, net contact or Sky Beta.

Gazebo exports are available through `tools/build_course.py` for the primary
PX4/ROS integration path. A MuJoCo-to-PX4 bridge would require deliberate design
and version checks; a body-wrench model must not be presented as PX4 SITL.

Obtain the organizer's aircraft and camera contracts and integrate sensors,
estimator and reviewed flight control before mission tests. Eventually validate
the same algorithms with the organizer's ROS/PX4 stack. A generic
airframe can be a clearly labeled proxy; it is not Sky Beta qualification.
Use ordinary ROS sensor interfaces; simulator truth is allowed only in a separate
independent evaluator/recorder, never as mission state or a fake estimator fix.

Host probe: WSL Ubuntu 22.04 exists but no ROS/Gazebo/colcon or PX4 checkout was
found there initially. Docker engine was unavailable and GPU access remains unverified.
Additional inspection found Gazebo Classic 11 and ROS Noetic in Ubuntu 20.04;
the packager now emits a compatibility scene for static geometry inspection.
The local proxy has since been installed in that WSL distribution, with Gazebo
8.15.0 (Harmonic), ROS Humble, PX4 v1.16.0 and release/1.16 messages. Source hashes
and package versions are recorded in artifacts/runtime/runtime-lock.json.
This is explicitly a local proxy pending organizer assets/versions. It does not
modify the managed lab or issued aircraft. See sim/runtime/README.md.

Sensor checks have received actual 640x400 mono8 stereo, RGB8, 32FC1 ideal depth,
CameraInfo, IMU and clock messages. Stereo delivered ~15 Hz and IMU/clock ~240 Hz
in simulation time in the first check. Software rendering and build load slowed
wall time; RGB/depth delivered fewer frames than CameraInfo and need investigation.
CameraInfo reports approximately fx=381.36, cx=320, cy=200 and a 0.075 m baseline.
Optical TF, latency and estimator validity are not established by these values.

The initial camera snapshots revealed an opaque CAD net roof casting unrealistic
whole-course shade. Gazebo exports now separate all 33 visual groups, assign
published MTL diffuse colors explicitly, and make the five net visuals transparent
and non-shadowing while keeping original collision geometry. Initial dark images
and the intermediate material checks are preserved. The proxy's start yaw is
configurable and now faces the first corridor (+Y), rather than a blank adjacent
wall. This is scene initialization, not a hard-coded flight route or a pose fix.

## Route requests in chat

Example: "Try the window instead of Gate B, keep the maze conservative, and disable
return." Translate to a named config under `configs/`, preserving the exact prompt,
structured route choice and qualified limits. Review geometry and checkpoint order
before execution. Configs currently save intent only; no planner consumes them yet.
In the eventual implementation freeze a hash of the config when the run starts.
Do not change route, modes or parameters midrun. Changes to qualified speed limits
require their own measurements and bounded mission proposal. Compare route variants
with all other settings held constant and preserve both failures and successes.

## Flight approval and fault behavior

AGENTS.md requires: "Explain a flight plan and obtain the team's approval before
arming." Prepare a concrete bounded mission first: aircraft, route, altitude,
speed/acceleration bounds, duration, return choice, estimator prerequisites and
abort behavior. General implementation approval is not approval of unspecified
future flights. Inspect fresh READY/landed/disarmed/estimator reports before start.

One controller only. Stop publishing when authority is lost or required telemetry
is stale; do not let an independent heartbeat keep an unhealthy mission alive.
Hold/brake requires valid estimates. Preserve configured PX4 failsafes; no guessed
position fix or weakened grader. After keyboard RC: land, release RC and use Reset
world before automation. Reset is outside mission code. In the managed lab use
Run saved file for its independent grading/recording; terminal execution is not a
managed run. Local simulator evaluation needs its own recording/evaluator.

## Benchmark protocol and recordings

First qualify takeoff/hover/land, straight braking, turns, each course stage and
then complete Gate B. Run existing companion tests before moving tests. For every
strategy use matching scene version, aircraft, sensing/estimator, stage logic,
limits, seed and initial conditions. Freeze weights and prompts for the baseline.
Initially repeat at least five full trials per runnable strategy; expand to a
larger held-out robustness set once stable. Report sample count and uncertainty,
not just a fastest successful clip. Test lighting, layout, delay and sensor loss.

Capture observer video plus onboard view with simulation timestamps, ROS bags,
PX4 ULogs, stage/authority events, contact events, CPU/GPU load and model latency.
Observer/truth recording belongs solely to evaluation. Preserve failed videos.
Store artifacts outside Git and save portable metadata with hashes. Never include
credentials. Export a playable MP4 with original source and FFmpeg command retained.

Each result must include code revision/content hashes, input/model/config hashes,
software/aircraft/scene versions, seed, requested/actual rates, approval reference,
qualification/reset status, raw launch/finish/checkpoint simulation timestamps,
elapsed wall time, touches, penalty-adjusted time, outcome, failure cause, route,
return outcome, recorder/evaluator versions and relative paths/hashes for evidence.
Read timing from actual simulated events, not video wall duration or planner ETA.
An unsuccessful trial gets no finish time; retain its last checkpoint time.
A return has separate timing and must not overwrite outbound finish time.

Deliver a comparison table with finish rate, successful-run median/best times,
penalty-adjusted times, checkpoint distribution, contacts, inference latency and
recording links. Mark unavailable models BLOCKED and untested strategies NOT RUN.
The milestone is reached only when each runnable baseline has real recordings
and measured results, with blocked models explicitly explained.

## Work sequence and acceptance

- [x] Read local context, supplied rules and course geometry.
- [x] Save this plan and sources; inspect runtime availability.
- [x] Implement and test scalar acceleration/braking planning and experiment inputs.
- [x] Generate static scene packages and validate XML/input hashes.
- [x] Compile/render imported course in the optional MuJoCo sandbox.
- [x] Select organizer-compatible Gazebo/PX4 as the end-to-end benchmark target.
- [ ] Obtain/pin the organizer's actual PX4, Gazebo and ROS release combination.
- [x] Install and lock a clearly labeled local WSL proxy stack; build PX4/messages.
- [x] Observe actual Gazebo sensor messages through ROS during bounded checks.
- [x] Add read-only timing diagnostics and conservative observed-grid A* component.
- [x] Add generic dynamics/actuator/sensor definitions and hole/contact smoke checks.
- [x] Record clearly marked frozen-state scene preview, with separate performance metadata.
- [x] Load course/proxy in Gazebo and inspect initial onboard images.
- [ ] Finish full collision/rendering/dimension qualification for the Gazebo scene.
- [ ] Obtain starter source, aircraft dimensions/dynamics and calibration.
- [ ] Establish ROS/PX4/sensors/VIO with fresh estimator evidence.
- [ ] Implement observed goals, local map, path planning and mission state machine.
- [ ] Propose bounded flight, obtain approval and qualify baseline stages.
- [ ] Pin/load OpenFly and AeroVLA, benchmark offline inference and native adapters.
- [ ] Qualify full-course classical and runnable VLA experiments.
- [ ] Record repeated runs and produce measured comparison report.
- [ ] Recheck OpenFlyV2 release, qualify window/return, then optimization/RL.

## Independent work while positioning is validated

These tasks can proceed without a flying vehicle. None is a flight qualification.

| Workstream | Next deliverable | Validation before integration |
| --- | --- | --- |
| Positioning (critical path) | Shadow OpenVINS run on actual stereo/IMU topics | Initialization, advancing estimates, frames, covariance and latency; subsequently motion accuracy |
| Perception and mapping | Depth-to-observed-occupancy component | Synthetic calibrated depth cases, invalid depth handling, free/occupied/unknown distinctions and footprint clearance |
| Mission behavior | Ordered checkpoint state machine and route preferences | Replay observed events; reject skipped or ambiguous checkpoints and stale observations |
| Recording and evaluation | Timestamped image recording and run evidence manifest | Decode output video, preserve source timestamps; no finish time without actual checkpoint events |
| VLA feasibility | Pin available upstream inference contracts and dependencies | Offline image/prompt inference latency and action units; flight adapter follows review |
| Simulator health | Diagnose magnetometer measurement and frame/unit contract | Compare pinned sensor and PX4 bridge sources; preserve safety checks |

Prioritize mapping and mission logic alongside positioning. Do not run multiple
Gazebo workloads or large-model inference during timing qualification on this
laptop. OpenFlyV2 remains recorded as blocked pending a release check; the model
registry is not evidence that any VLA has been downloaded or run.

## Sources and provenance

Competition text/hardware specifications supplied in this conversation; original
instructions: AI_CONTEXT.md, CLAUDE.md, AGENTS.md, CONNECT_FROM_LAPTOP.md;
geometry: dronehacks-course-v3-obj/README.txt and OBJ/MTL. Check event updates.

- https://github.com/SHAILAB-IPEC/OpenFly-Platform
- https://huggingface.co/IPEC-COMMUNITY/openfly-agent-7b
- https://github.com/XuPeng23/AeroVLA
- https://huggingface.co/XuPeng23/AerialVLA
- https://github.com/Ideal-ljl/AeroBrain
- https://arxiv.org/abs/2610.39287
- https://airvla.github.io/
- https://github.com/uzh-rpg/agile_autonomy
- https://docs.px4.io/main/en/ros2/offboard_control
- https://github.com/mavlink/mavros/blob/ros2/mavros/src/lib/ftf_frame_conversions.cpp
- https://docs.luxonis.com/hardware/platform/features/depth
- https://nvidia-isaac-ros.github.io/v/release-3.1/repositories_and_packages/isaac_ros_visual_slam/index.html
- https://arxiv.org/abs/1812.03201
- https://www.nature.com/articles/s41586-023-06419-4
- https://mujoco.readthedocs.io/en/stable/python.html
- https://mujoco.readthedocs.io/en/stable/computation/index.html
- https://github.com/SarahWeiii/CoACD
- https://docs.px4.io/main/en/simulation/
- https://docs.px4.io/main/en/sim_gazebo_gz/


## Latest diagnostic correction and status (November 4)

The v1?v3 disarmed VIO fixture rail was misaligned and contacted the north net;
their previously reported free-space errors are invalid. The corrected v5 run
used local +X at the verified pi/2 fixture yaw and software rendering after the
GB10 hardware renderer failed to access DRM. Its accepted interval passed the
independent Sky Beta envelope screen with zero sampled contacts. A one-time
aligned offline comparison over 3,052 samples measured 2.79 cm RMSE and 3.78 cm
maximum error. This is a slow sub-metre fixture motion, not course-speed VIO
qualification. The A* flight failure remains valid.

A provisional route guard is implemented using OpenVINS' standard
`/vio_shadow/points_slam` PointCloud2 stream. The v6 disarmed diagnostic captured
443 clouds; the contact-screened segment had 270 clouds and a 2.23 sim-second
longest span below 10 finite points. The 1.0 m/s v7 disarmed rail capture had zero sampled envelope contacts and a
1.82-second longest low run below 8 features. The earlier 10/3-second candidate
would have faulted on a clean 3.17-second low run below 10, so the guard now uses
8 features / 2.5 continuous seconds and requires 3 continuous adequate seconds
to arm. Historical A* evidence contains internal update counts, not this cloud
stream; calibration remains provisional. Next: exercise the adapter read-only
on recorded/live course imagery, then run a bounded course-speed estimator
check before another armed attempt. Simulator truth
stays offline. No complete outbound course run, finish time, or return bonus is
established. Evidence: [v5](artifacts/gb10-sky-beta-vio-quality-v5/README.md),
[v6](artifacts/gb10-sky-beta-vio-quality-v6/README.md),
[v7](artifacts/gb10-sky-beta-vio-quality-v7/README.md).
