# Sky Beta model and full-mission integration

Latest qualification status (November 5): bounded takeoff/landing works, but
the first corner qualification requested Land on its first straight after
feature loss. It landed/disarmed; it did not finish the corner or course.
Ground-height VIO passes do not transfer to flight-height net imagery. New
measured-stereo/public-map estimation is being evaluated offline and in a
read-only live shadow; it has not replaced flight odometry or its watchdog.
The latest full local suite passes 334 tests and 52 subtests. See `PLAN.md`
for current evidence and remaining work; older counts below are historical.

## Current estimated flight model

Suraj authorized estimates because organizer propulsion measurements are not
available. `configs/sky_beta_estimates.json` records each assumption and its
basis: 7-inch three-blade props, 6 g each, 35 g wiring, 5 g rangefinder/bracket,
10 N maximum thrust per motor, 1500 rad/s maximum speed, and 35/50 ms motor
rise/fall constants. These are engineering estimates, not measured Sky Beta data.
Sensitivity variants are declared but have not yet been flown.

`sim/sky_beta_flight_model.py` adds these components and active propulsion to
the actual CAD body. Total estimated mass is **1.25730071 kg**. Its COM in the
original CAD frame after conversion to FLU is
**[-0.01255489, 0.00015134, 0.06583886] m**. Inertia at that COM, kg m²:

```
 0.0073449833  -0.0000078210   0.0000394771
-0.0000078210   0.0075918514   0.0000081547
 0.0000394771   0.0000081547   0.0121206406
```

The current executable artifacts are under `artifacts/sky-beta-estimated-v4/`.
Use `airframe-contract.json` and the standard 4001 multicopter startup. The
contract replaces rotor geometry, thrust assumptions, sensor offsets and
physical output limits; safety checks remain intact. Armed idle is explicitly
estimated at 150 rad/s; disarmed output stays zero. Hover command accounts for
native quadratic thrust linearization and idle-to-maximum speed scaling.

The real stereo/IMU OpenVINS pipeline and heading alignment have passed
disarmed native Offboard preflight on this model. The bridge compensates the
CAD-derived IMU lever arm, including rotational velocity and covariance.
The new mission adapter also passed its live read-only check: 37.54 seconds
of continuous idle health, no control publishers and no commands. A float32
JSON logging failure from the preceding attempt is preserved, then corrected.
The latest full suite passed **231 tests** before a later watchdog test was
added; its focused 21-test selection passed. A full rerun is pending.

The user-requested simulated launch/fly/land qualification passed in attempt v2:
0.5 m rise at 0.25 m/s, 2 s hold, native Land, observed landing and disarm,
with a 0.03 m XY and 0.1 rad heading bound. The controller observed native Land
ACK/mode and actual landed/disarmed reports. ULog independently confirms takeoff
and landing detection, no ESC failure, **15.612497 simulated seconds from arm to
disarm**, maximum estimated rise **0.4757 m**, XY displacement **0.01922 m** and
heading change **0.00265 rad**. These are onboard-estimator measurements, not
ground-truth accuracy claims. Native parameter audit matches every contract
parameter. Evidence: `artifacts/gb10-sky-beta-hover-v2/native-measurements.json`.
The first attempt (`gb10-sky-beta-hover-v1`) armed, then correctly failed the
native ESC check and disarmed before takeoff. ULog shows ESC online flags 15
but armed flags 5: rear motors were stopped at the previous zero armed minimum.
The native failure detector requires all armed flags after 300 ms. The earlier
4099 profile enforcing zero was therefore a modeling error, despite passing
its parameter audit. It is preserved but superseded; no ESC check was bypassed.
The corrected v4 model has spinning armed idle and a recalculated hover command.
Attempt v2 repeated the unchanged bounded plan in a fresh world. Model build v3
also preserves a failed preparation using a template without magnetic metadata;
v4 uses the complete previously qualified sensor/environment template.
The original observer angle hid the launch lane behind a wall. Attempt v3 is
repeating the same flight with an overhead display-only camera for a useful
recording. The original occluded video is preserved; it is not visual proof of
the hover. No camera view is supplied to the flight algorithm as ground truth.

The observer stream records actual Gazebo frames. This is estimated SITL only;
even a pass does not establish a complete course finish or hardware dynamics.

## Latest A* route attempt and VIO follow-up (November 4)

The first classical A* stop/turn/go route trial used the estimated Sky Beta
model and the estimated strand-rendering scenario. The collision geometry and
4 cm measured-path guard were unchanged. The flight took off; navigation began
at 115.129 s. The independent scene-pose stream shows the aircraft contacted
`collision_g09_Net_side_north_c0` at 127.225 s. The onboard tracking guard
requested native Land at 128.471 s. PX4 accepted Land and reported landed and
disarmed by 133.671 s. The supervisor state is `ABORT_LANDED`; this is not a
course finish, and no return was attempted.

The separate offline comparison of native odometry with the Gazebo scene shows
the position estimate lagging by 0.670 m at 128.462 s, mostly along the course
axis. OpenVINS logged zero features in 837 of 928 MSCKF update cycles (maximum
13 features); the final recorded track histogram was blank. A sampled tilted
Sky Beta envelope screen found 492 contact samples and up to 0.1056 m
penetration at the unchanged net collision geometry. Simulator truth was not
fed into the estimator, flight controller, or mission. This failure indicates
that the current estimator-health checks did not catch the VIO degradation
before a collision.

Evidence is grouped in
[`artifacts/gb10-sky-beta-astar-net-v1-review/README.md`](artifacts/gb10-sky-beta-astar-net-v1-review/README.md).
Its 134.6 s, 15 fps overhead video is a fixed-rate viewing aid from
`/camera/observer/image`; it is not a competition-time recording. The actual
competition clock and finish time were not established.

The first three fixture attempts were not valid free-space comparisons; a
corrected software-rendered v5 run is now complete. This diagnostic identifies
usable offline motion evidence but does not establish a live estimator-quality
gate for flight. Before another course run, implement and inspect a conservative
live quality signal, then re-qualify it in a bounded non-course SITL run. The
route should not be retried until that gate and the VIO issue are addressed.

The first three disarmed VIO fixture runs (`gb10-sky-beta-vio-quality-v1` through
`v3`) are invalid as free-space odometry accuracy checks. Offline screening of
the recorded Gazebo poses found that the fixture's local +Y rail axis, combined
with its inherited pi/2 yaw, moved world -X into the north net. The v2 and v3
paths had repeated aircraft-envelope contact; their previously reported 0.0361 m
and 0.0257 m VIO-to-scene errors must not be interpreted as free-motion VIO
accuracy. The feature-track and image recordings remain useful evidence of
sparse/variable tracks during those contact-contaminated motions. A corrected
fixture uses local +X, which maps to course-forward world +Y at the verified
fixture yaw. The v4 attempt failed closed because hardware rendering could not
access the GB10 render device and produced no image frames. The software-rendered
v5 disarmed capture produced advancing camera/VIO streams. Its sim 13.94?49.5 s
interval passed the independent envelope screen with zero sampled contacts; over
3,052 VIO samples the offline position comparison had 2.79 cm RMSE and 3.78 cm
maximum error. This is a short, slow fixture motion, not course-speed VIO
qualification. Post-release floor settling was excluded. See
[`artifacts/gb10-sky-beta-vio-quality-v5/README.md`](artifacts/gb10-sky-beta-vio-quality-v5/README.md).
The v6 disarmed capture additionally recorded 443 actual `/vio_shadow/points_slam`
PointCloud2 samples. In the screened interval it recorded 270 clouds, median
19 finite points, 53 below 10, and a longest low interval of 2.23 sim seconds.
Its screened VIO comparison measured 1.23 cm RMSE / 2.28 cm
maximum. The 1.0 m/s disarmed v7 capture found that 10 features / 3 seconds would trip
on a contact-free interval. The route guard was adjusted to 8 features / 2.5
continuous seconds; v6 and v7 recorded paths pass that persistence screen. The
A* flight did not record this PointCloud2 stream, so the guard remains provisional
until exercised on course imagery. See
[`artifacts/gb10-sky-beta-vio-quality-v7/README.md`](artifacts/gb10-sky-beta-vio-quality-v7/README.md).
The A* course failure remains valid.


The passive-model notes below preserve earlier import work. Their missing
actuator status applies to those passive artifacts, not the executable model.

The user's `Downloads/sky-beta-cad.zip` is preserved under `assets/sky-beta-cad/`.
It contains STEP assembly, an assembled camera-mount STL and the organizer README.
The README specifies millimetres, Z up, camera/front at CAD -Y. Propellers,
wiring and the downward rangefinder are explicitly absent.

## Imported model

[Preview](artifacts/sky-beta-passive-v2/sky-beta.png).

`tools/import_sky_beta_cad.py` reads the actual STEP assembly and density-property
graph. It converts millimetres to metres and CAD -Y to FLU +X with a proper
rotation, preserving Z up. Named component assemblies are integrated once,
including unnamed internal solids; the separately supplied STL mount is added
once with the document's PLA density. CAD masses include calibrated component
weights and estimated frame/mount material densities, not a scale measurement.

Current derived values (excluding omitted equipment):

- Mass: **1.19330071 kg**.
- COM relative to original CAD origin, FLU: **[-0.01295855, 0.00015945, 0.06591520] m**.
- Inertia about that COM, in FLU, kg m²:

```
 0.0067473055  -0.0000078978   0.0000392811
-0.0000078978   0.0071221142   0.0000081692
 0.0000392811   0.0000081692   0.0110662498
```

Four motor shaft XY positions are derived from the actual component geometry
relative to COM. The layout is asymmetric: front X≈+0.0961 m, rear X≈-0.1345 m;
front Y≈±0.1597 m and rear Y≈±0.1351 m. Motor instance names are retained for
traceability; physical body coordinates, not the CAD words "left/right", must
determine PX4 motor numbering and torque signs. Shaft rotation directions are
not encoded in this CAD and must be supplied.

`sim/sky_beta_model.py` exports a new Gazebo SDF and MuJoCo scene, replacing the
entire X500 proxy body. Its body origin is COM; all meshes are translated by
the negative CAD COM. Explicit CAD-derived full inertia is used. Each component
gets a conservative convex collision hull. No X500 mass, inertia, motor arms
or force constants are inherited. Dense visuals retain their triangles: OBJ
is used where MuJoCo's STL facet limit requires it.

Artifacts:

- `artifacts/sky-beta-cad-v2/cad-properties.json`: original input hashes,
  component masses, bounds, COM/inertia, units and omissions.
- `artifacts/sky-beta-model-v2/model.sdf`: Gazebo rigid body.
- `artifacts/sky-beta-model-v2/course-sky-beta.xml`: MuJoCo course + Sky Beta.
- `artifacts/sky-beta-model-v2/airframe-contract.json`: explicit missing dynamics.

**Motor plugins are disabled, not filled with X500 constants.** An accurate
looking mesh does not supply a propeller model, thrust curve, motor/ESC lag,
spin direction, wiring mass or rangefinder mount. Those values remain missing.
The pending user question asks for propeller diameter/pitch, flight-ready mass
and any organizer motor/dynamics specification. A manufacturer curve for a
different "2807 1500KV" brand is not a substitute for this YSIDO/prop combination.

## Validation completed

The rigid body loaded in MuJoCo with the expected mass and principal inertia.
With no actuators, the first 0.1 s drops 0.05108 m versus analytic continuous
freefall 0.04903 m (the difference matches finite-step integration). After 2 s,
it rests with four contacts and COM Z≈0.06577 m. This is a passive drop/contact
check, not flight. Exact report: `artifacts/sky-beta-passive-v2/result.json`.

A bounded GB10 Gazebo asset-import/drop diagnostic is in
`sim/sky_beta_asset_check.py`; it runs no PX4 and sends no motor/flight commands.
Keep its independent scene-pose evidence out of onboard code.

Import failures remain preserved: CAD v1 stopped on the Jetson's unnamed
COMPOUND child until material ownership was resolved at its component assembly;
model/passive v1 hit MuJoCo's dense-STL limit, corrected by lossless OBJ visuals
in v2. Neither failure was suppressed or counted as a successful flight.

## Mission code implemented

`dronebench/course_mission.py` adds a supervisor around the existing telemetry
and ACK validation. It requires 30 continuous wall seconds of idle landed /
disarmed health, >1 s actual Offboard prestream, native ACK plus actual mode,
then 30 continuous wall seconds of native Offboard preflight before arming.
It implements bounded takeoff, navigation handoff and Land ACK/mode followed
by actual landed **and** disarmed telemetry. It stops publication on stale data,
identity/reset changes or authority loss. Stopping publication is not landing.

`dronebench/teammate_policy.py` imports the original teammate `PolicyRunner`,
rather than rewriting PPO. It adds input/output contracts and launch-frame
calibration using actual PX4 pose/heading, a reviewed physical launch alignment
and a calibrated floor range. NED yaw and course yaw have opposite signs:
`north_heading_course = physical_heading_course + heading_NED`.
It also reads their recorded trajectory, with explicit reference time stretching
for a distinct classical PX4 position-controller baseline.

`tools/course_flight.py` is the ROS adapter, default read-only. It supports the
bounded hover, learned-path position tracking and the original PPO body-rate
mode, with native topics, causal source timestamps, ACK filtering and command
ownership checks. These new paths are unit-tested/prepared, **not live-flight
validated**. It refuses execution while the Sky Beta actuator contract is
incomplete. Sample settings are `configs/sky_beta_hover.json` and
`configs/sky_beta_outbound.json`; missing range extrinsics are explicit nulls.

First proposed qualification is a 0.5 m rise at 0.25 m/s, 2 s stable hold, Land,
and observed disarm, within 0.03 m XY / 0.1 rad heading bounds. This has not
been approved or run. After it passes, the first outbound uses the learned
trajectory at four times its original reference duration, then landing. Actual
measured time must include takeoff/landing and use independent events. Direct
PPO is a separate later comparison; scaling PPO actions is not equivalent to
retiming the reference path.

Return is implemented at the supervisor/reference level: a fresh preflight and
takeoff after confirmed outbound landing/disarm, with a separately checked
reverse reference rather than blindly reversing the PPO. Its execution is
deliberately pending independent successful outbound evaluation and return-route
clearance. The ROS command currently rejects auto-return on the first outbound
qualification. No return bonus is claimed by merely touching the pad or by the
7.04 s airborne-to-hover result.

## PX4 infrastructure correction

Disarmed diagnostic v3 confirmed native mode ACKs, then rejected a position
sample 0.00417 s ahead of the latest ROS clock callback. `CausalInbox` now holds
source samples until their original timestamp is causally available; it never
changes timestamps or refreshes old data. Regressions and queue overflow latch
failure. No source/arrival freshness bounds were loosened.

Fresh v4 passed: 30.02 s idle health, accepted mode ACK and actual Offboard,
30.03 s native preflight true, 600 heartbeat samples, landed/disarmed throughout.
Its report is `artifacts/gb10-disarmed-offboard-v4/offboard-disarmed-preflight.json`.
This old-proxy infrastructure diagnostic is **not Sky Beta flight qualification**.

## Live SLAM feature observer check

The route adapter now observes actual `/vio_shadow/points_slam` clouds before
any mission readiness decision. A disarmed 1.0 m/s rail-motion fixture on the
GB10 produced 361 feature callbacks from sim 14.38 to 61.88 s. With the
provisional gate (8 finite XYZ features, 2.5 s maximum continuous low count,
3 s startup observation, and 0.5 s source/arrival freshness), it reached READY
at sim 19.24 s and ran to shutdown with no quality fault. Source age ranged
from floating-point zero to 4.17 ms. No control publishers or flight commands
were created. Evidence: `artifacts/gb10-sky-beta-vio-quality-v10/`.

The preceding v9 run caught a numeric edge case: equal ROS/message timestamps
could subtract to a roughly -7e-15 s float epsilon and be misclassified as a
future stamp. `dronebench/vio_quality.py` now tolerates up to 1 microsecond of
representation error while still faulting a genuinely future-dated stamp and
retaining the 0.5 s stale-source bound. v9 is preserved in
`artifacts/gb10-sky-beta-vio-quality-v9/`; v10 is the post-fix live check.
This validates the observer in a disarmed fixture only. It does not prove
estimator accuracy during powered flight, qualify the estimated propulsion
model, or complete the obstacle course.

## Remaining before a full recorded Sky Beta flight

1. Complete propeller, motor/ESC and omitted-mass data; model rotor inertia/force,
   response and torque signs, then generate matching PX4 allocation/calibration.
2. Add camera/IMU/range sensors with reviewed extrinsics. Actual Sky Beta's IMU
   does not coincide with COM: the existing zero-lever-arm EV bridge must not
   silently be reused. Provide complete lever-arm compensation or correctly
   configured PX4 external-sensor offsets, with covariance/velocity checks.
3. Validate estimator accuracy, image quality and PX4 fusion during bounded
   Sky Beta motion. The feature observer has passed a disarmed live check, but
   powered-flight estimation remains unqualified.
4. Obtain approval of the concrete first flight, then record hover/landing and
   outbound trials. Preserve failures; never relabel a replay as a flight.
5. Once outbound is independently successful, qualify and record the return.

A* remains useful for alternative routes or changed layouts. Shortest distance
is not minimum time: clearance, turn radius, acceleration and braking belong in
trajectory optimization. The existing PPO is a useful fast baseline; its current
7.04 s result is not evidence that the route or controller is globally optimal.
