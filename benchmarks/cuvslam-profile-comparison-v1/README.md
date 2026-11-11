# cuVSLAM through the organizer (Sky Beta) pipeline: stereo-only vs stereo + IMU

Two recorded sim flights on 2026-11-07, both through the drone-faithful profile
(`SKYBETA_PROFILE=1`: estimator → `skybeta_vio_relay` → organizer
`px4_odom_bridge` → PX4 EKF2 with EV_CTRL=11/HGT_REF=3, organizer
`flight_interface`, our `skybeta_team_nav`, sim pilot with stick stand-in),
k12 reference, `RECORD_BAG=1` (stereo + IMU + clock mcap kept on GB10).

| run | estimator | armed → nav | what happened | arm→disarm | video |
|---|---|---:|---|---:|---|
| v107 | cuVSLAM 17.0.1 **stereo-only** (as on the real drone) | 1.2 s | Offboard, takeoff; at 11.6 s cuVSLAM's rotation disagreed with the gyro by 2.8–3.5 rad/s (true yaw rate ≤ 0.3), our consistency gate withheld the pose, team node landed on "stale odometry" at 16.4 s | 20.0 s | `../gb10-sky-beta-course-v107/flight-video/flight.mp4` |
| v118 | cuVSLAM 17.0.1 **Inertial mode** (stereo + `/imu/data`) | 1.0 s | same start; gate tripped at 5 s into navigation, 339 of 2140 frames reported lost tracking, team node landed at 13 s; PX4 held a blind-land descent until 67 s | 67.2 s | `../gb10-sky-beta-course-v118/flight-video/flight.mp4` |

Neither flight passed a gate. The OpenVINS stand-in in the same profile (v98, same
day) armed, took off and navigated until a 12 cm height disagreement, which the
EV_CTRL=11/HGT_REF=3 change fixed for v107/v118 (both climbed cleanly).

## Offline accuracy on the same recording (v81 bag, 243 s, 18.4 m of path)

`sim/runtime/replay_cuvslam.sh` + `tools/eval_estimator_replay.py` against truth:

| mode | SE3-aligned xy RMS | drift (first-frame frame) | gate would trip at |
|---|---:|---:|---:|
| OpenVINS stereo-inertial (reference) | 2.7 cm | 1.2 % | never |
| cuVSLAM stereo-only | 48 cm | 27 % (climb scale 0.27 of 0.84 m) | 11 s |
| cuVSLAM Inertial, sim IMU noise model | 8.7 km (diverges) | — | 14 s |
| cuVSLAM Inertial, IMU noise ×100 | 76 m (diverges) | 425 % | 12 s |

Inertial mode needed two node fixes to run at all (cuVSLAM requires IMU samples
registered per frame up to the frame stamp, strictly increasing; GB10 also needed
cusolver 12.4 for the inertial solver). Even then it diverges: the IMU/rig
convention or the near-zero simulated IMU noise is still wrong for cuVSLAM's
inertial filter. Stereo-only is the mode the organizer stack runs, and it is
also the one that is usable here, but it is 18× worse than OpenVINS on the same
images and its rotation is wrong enough during the first yaw that the
consistency gate (which protects the drone from a bad pose) lands the flight.

## What differs, then

- **With IMU**: worse, not better. cuVSLAM's inertial fusion in this sim blows
  up within seconds; on the real drone the organizer keeps IMU fusion off
  (`enable_imu_fusion: false`), consistent with this.
- **Without IMU**: tracks, but with large rotation error during in-place yaw
  (the start turn), which both flights hit immediately.
- OpenVINS remains the estimator to fly with in sim (`ESTIMATOR=openvins`); for
  the real drone the organizer's cuVSLAM is fixed, so the reference must avoid
  in-place yaw rates it cannot follow (see `artifacts/failure-map-v1/README.md`).

Node: `tools/cuvslam_vo_node.py` (`--inertial`, `--imu-noise-scale`); runner
knob `CUVSLAM_INERTIAL=<kalibr_imu_chain.yaml>` with `ESTIMATOR=cuvslam`.
