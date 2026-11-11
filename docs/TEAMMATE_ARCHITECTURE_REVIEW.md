# Teammate PPO baseline: reproduced and adopted

Source: https://github.com/jakefromstatefarm00/dronehacks2026

Pinned commit: `a03b7a6a7e164b37e095bbe7555451235f675efe`.
Checkout: `teammate-review/dronehacks2026`. Source files were not modified.
The tracked archive `dronehacks_rl.zip` includes trained SB3/ONNX policies,
normalization, training logs and recorded trajectories absent from the loose
`runs/ppo` directory. Selected assets were extracted to `artifacts/teammate-bundle-v1`.
Course OBJ text is identical to our supplied original after CRLF normalization.

## What actually works

We ran the original environment tests: **4 passed**. Reused the original
`evaluate.rollout`, `drl` simulator/observations/gates and trained policy, rather
than recreating their algorithms. SB3 version matches the artifact: 2.9.2.

| Fresh test | Result |
|---|---|
| Nominal airborne start to pad hover | 1/1, 7.04 simulated seconds |
| Randomized episodes, seed 1 | 256/256, best 6.84 / median 7.08 / worst 7.30 s |
| Original ONNX deployment-core loop, headings default / 0 / 2.3 rad | 3/3, all 7.04 s |

Raw outcomes, checkpoint hashes and all trajectories:
`artifacts/teammate-reproduction-v2/`. Policy inference is actual SB3/ONNX;
these are their NumPy simulator results, not Gazebo/PX4 flights.

Video of the fresh nominal states:
[7.04 s replay](artifacts/teammate-fresh-replay-v1/rollout.mp4).
MuJoCo renders the saved NumPy states; it did not generate the flight dynamics.
The visible proxy/envelope is illustrative, not measured Sky Beta hardware.
The last logged pre-step state is 7.02 s; the evaluator's next terminal step is
7.04 s. The replay retains the source sampling rate (50 Hz, 352 frames).

## Does it actually follow the obstacle route?

The recorded center stays between 0.5964 and 1.1532 m above the floor. Walls are
1.7272 m. Even adding the bounding-sphere extent of the assumed 0.25 m-radius,
0.12 m-half-height cylinder keeps this proxy below wall height. No wall hopping
is present in this nominal trajectory.

Independent checks on the fresh trajectory found all nine configured planes
crossed in order: six maze/corridor targets, tunnel entry and exit, then window.
Window crossing occurs at 6.208 s, with tilted-envelope lateral/vertical
clearance approximately 0.122 / 0.217 m. At 5 ms interpolated samples, MuJoCo
reported zero penetrating contacts between that tilted cylinder and the convex
decomposition of the original course mesh. See `gates.json` and `validation.json`
beside the video. Decomposed-mesh sampling is not exact continuous swept-volume
or hardware qualification. The remaining 256 trajectories were evaluated by
the teammate's simulator; this independent mesh check covers the nominal one.

**This is not yet a full competition finish.** The timer starts at a 1 m hover
and stops at a slow hover near the pad, around 0.6 m high. It excludes takeoff,
touchdown, disarm, and any return. Configured gate crossings are not an official
grader. Actual Sky Beta extent, estimator errors, contact scoring and landing
still need independent qualification. A success can also coincide with a crash
in the environment's general `finished` flag logic; inspect both fields rather
than treating success alone as sufficient. Fresh nominal had no crash/miss.

## Architecture and what to reuse

Their pipeline is:

`known course + gate list -> shared 35-value state observation -> small PPO MLP
-> collective thrust / body rates -> PX4 rate controller`.

There is no A*, occupancy mapping, camera policy or VLA in this controller.
It assumes usable course-frame position/velocity/attitude and known gate poses.
Its learned action sequence already accelerates/brakes/turns; its tiny ONNX
artifact is 88,420 bytes, far more practical to investigate on an 8GB Jetson
than the 7B VLA checkpoints. Jetson latency has not been measured here.

Adopt their `drl/obs.py`, `drl/config.py`, training reward/curriculum,
`train.py`, `evaluate.py`, `export_onnx.py`, policy/normalization and deployment
core frame tests. Keep these upstream files pinned and reuse through an adapter.
Their recorded reference trajectory can also seed a classical PX4 trajectory
follower. Existing A*/observed-map work remains useful for route alternatives,
changed layouts and fallback planning, rather than duplicating their controller.

Retain our actual stereo/IMU VIO, external-vision alignment, telemetry freshness,
command ownership and evidence/recording infrastructure. Their ROS node needs
integration work before use: it checks callback age rather than source progress,
does not independently expire status, assumes odometry frame/quality, hardcodes
IDs, does not require command ACKs, and continues publishing a hover-rate stream
after leaving Offboard. Its HOLD selects AUTO_LOITER, whose native requirements
include global position in our pinned no-GNSS PX4 configuration. Resetting VIO
with the nose along the course does not establish geographic North alignment.
Do not run that node on an armed aircraft as copied.

The NumPy dynamics use prescribed first-order rate/thrust response and ideal
state with added noise. They do not simulate PX4 estimator/controllers, image
tracking loss, actual motors, battery or aerodynamic coupling. Radius/height are
assumed; training collision height is upright and independent of attitude.
Randomized success is valuable within this model, not a sim-to-real guarantee.

## Next work

1. Use the existing PPO policy as the first fast baseline, alongside its saved
   trajectory under classical PX4 tracking. No new training needed for this step.
2. Integrate the reusable policy core with the reviewed ROS/PX4 authority and
   source-freshness checks. Validate actual estimator/course frame alignment.
3. Validate measured proxy footprint and lower-speed behavior in Gazebo/PX4,
   then add takeoff, actual landing/disarm and independent checkpoint/contact
   evaluation. Obtain team approval for a concrete flight before arming.
4. Keep nominal, randomized NumPy and PX4/Gazebo times separate. Route prompts
   should change saved gate/route settings between trials; this trained policy
   is not qualified to follow arbitrary changed routes without evaluation.
5. Consider Isaac Lab for subsequent dynamics randomization/training, preserving
   the observation/action/reward contracts and policy baseline. Its official
   quadcopter example is a Crazyflie hover task, not a ready Sky Beta race model:
   https://isaac-sim.github.io/IsaacLab/main/index.html . Porting requires an
   explicit airframe, actuator/controller model, course and task implementation.
   Keep Gazebo/PX4 for transfer validation even if Isaac Lab trains the policy.

The first reproduction wrapper completed the nominal rollout but failed to
serialize NumPy terminal observations. That attempt remains under
`artifacts/teammate-reproduction-v1`; v2 explicitly serializes NumPy values.
An unrelated unfinished hover-controller refactor was interrupted; its source
was saved under `artifacts/hover-wip-interrupted-v1` and the previously tested
controller restored. The project suite then passed **185 tests**. No arming or
flight occurred. The latest disarmed PX4 diagnostic still refused qualification
on a transient position timestamp freshness failure; it is not a passed flight.
