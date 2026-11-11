# Next architecture: SLAM + mapped planning for a fast, robust course run

Status: proposal (Nov 6). Baseline to beat: **v51, 146.42 s arm→disarm**, 9/9
gates, zero contacts (`artifacts/gb10-sky-beta-course-v51/README.md`). Keep v51's
pipeline runnable as the fallback while this is built.

## 1. Where the current stack loses time and runs

Current pipeline: published-map A* → Bezier corner stops → offline heading DP →
offline speed profile → PX4 setpoints (+ accel/yaw feedforward). Estimator:
OpenVINS VIO. Nothing is perceived or replanned online.

| Bottleneck | Evidence | Cost |
|---|---|---|
| Path has 14 kinks (A* grid + corner splices) | v42/v43 overshot the kink at (-1.92, 2.585); kinks force full stops | ~30–40 % of nav time at speed |
| Feature loss in poor views / in-place turns | v41 (tube mouth turn), v52 (dropout at 35 s), v48 | most aborts |
| No drift correction | v40: 6.9 cm RMS / 15.4 cm max VIO error vs truth | forces 4 cm guard + 8 cm margins → caps speed |
| Yaw-rate cap 0.5 rad/s, takeoff cap 0.25 m/s | supervisor/mission validation | 13.6 s takeoff; slow corners |
| No online map | planner trusts the published layout blindly | unsafe if the real course differs; no replanning |

Organizer constraints that shape the design: one start command then autonomous;
all compute onboard (Jetson Orin Nano 8 GB); no GNSS/external positioning;
**published maps allowed, no data carried between runs** (code/parameter tuning
OK); organizer image uses **cuVSLAM**; online SLAM/mapping is encouraged.

## 2. Proposed architecture

```
 stereo + IMU ──► cuVSLAM (VIO + landmark map + loop closure, in-run only)
                     │ pose (odom frame, continuous)      │ map-frame corrections
                     ▼                                     ▼
              EV bridge → PX4 EKF2            map↔published-course alignment
                     │                         (start pose + wall/pad matches)
 stereo depth ──► local voxel/ESDF map (rolling, ~4 m) ◄──┘
                     │
   published map ──► GLOBAL: safe-flight-corridor + min-time polynomial
                     │        through ordered gates (offline at start, cached
                     │        in memory for this run only)
                     ▼
                LOCAL: B-spline replanner on ESDF (EGO-Planner style),
                       perception-aware yaw, 10–20 Hz
                     ▼
         supervisor: guards (tracking, VIO health, freshness) → PX4
         TrajectorySetpoint pos+vel+acc+yaw/yawspeed (existing, tested)
```

### 2.1 Estimation: cuVSLAM instead of OpenVINS
- Matches the organizer image and the Jetson deployment target (GPU front end
  frees the Orin CPU).
- Keep **two frames**: continuous `odom` (fed to PX4, never jumps) and `map`
  (loop-closure corrected). Corrections move the *reference*, not PX4's state
  (rule from PLAN.md: relocalization must not jump the trajectory reference).
- Anchor `map` to the published course once per run from the known launch pose
  plus observed structure (walls/pad), so the published plan and the live map
  agree. No stored maps between runs.

### 2.2 Mapping: online local map from stereo depth
- Rolling 3D voxel grid (occupied / free / unknown kept distinct — the window,
  tube and walls need height), ESDF for distances. Building blocks exist:
  `dronebench/depth_mapping.py`, `wall_fusion.py`, `map_vio.py`.
- Uses: (a) verify the corridor ahead matches the published map, (b) local
  replanning around anything unexpected, (c) wall-relative drift checks.

### 2.3 Global planning: corridors + time-optimal trajectory (replaces A* + corner stops)
- From the published map: seed topology with A*, inflate convex free regions
  (safe flight corridors) along it, then optimise one smooth piecewise
  polynomial (MINCO/min-snap with time allocation) constrained inside the
  corridors and through each gate opening in order, with v/a/jerk and
  yaw-rate limits.
- Removes the kinks and corner stops; corner radii become as large as the
  free space allows. Prototype in progress: `tools/plan_corridor_path.py`
  (offline, audited by the unchanged clearance + gate checks).

### 2.4 Local planning: replan on the live map
- Gradient-based B-spline replanning (EGO-Planner style) against the ESDF,
  warm-started from the global trajectory; 10–20 Hz; falls back to the global
  plan when the map agrees.
- Heading: keep the perception-aware view score, evaluated on the *live* map
  (texture/feature density from the current frames) instead of only the
  published geometry.

### 2.5 Control and supervision (keep)
- Existing supervisor, guards and PX4 feedforward stay. Re-derive the tracking
  guard from measured estimator error: with loop-closed drift ≪ 15 cm the
  clearance budget can fund speed instead of drift. Guard changes are made
  only from measured evidence and documented, never to make a run pass.

## 3. Milestones (each with a measurable exit gate)

| # | Work | Exit gate |
|---|---|---|
| M0 | Speed ladder on current stack (f5/f9 refs, ~40 s nav) | ≥1 completion; know the speed where VIO/tracking breaks |
| M1 | Corridor + min-time global trajectory (offline) | 0 kinks; audits pass; nav reference ≤ 30 s at ≤ 1.5 m/s |
| M2 | Fly M1 on current OpenVINS stack | 3/3 completions; truth eval 0 contacts |
| M3 | cuVSLAM integration (laptop x86 + 5070 Ti first, then GB10) | hover + v51 reference completes; drift vs truth < v40's 15 cm max |
| M4 | Online voxel/ESDF map + map↔published alignment | map matches published walls within 5 cm; runs at 15 Hz on target compute |
| M5 | Local B-spline replanner on live map | completes with a moved obstacle not in the published map |
| M6 | Faster takeoff/landing + guard re-derivation from M3 data | total arm→disarm 30–40 s, ≥ 4/5 success |
| M7 | Jetson Orin Nano budget check | full stack within 8 GB / real-time |

Run M0–M2 now (no new dependencies); M3–M5 in parallel by different
teammates since they share only the pose/map interfaces.

## 4. Interfaces (so people can work in parallel)

- Pose: `nav_msgs/Odometry` in `odom` (continuous) + `map`→`odom` TF.
- Map: voxel/ESDF service `distance(xyz) → (d, grad)`; occupancy query for audits.
- Plans: the existing 11-column reference CSV (`t,x,y,z,vx,vy,vz,qw,qx,qy,qz`)
  for global plans; local replans published as the same fields in memory.
- Every plan passes `tools/check_sky_beta_reference.py` (8 cm) and
  `tools/audit_scene_gates.py` before flight; every flight gets
  `tools/evaluate_flight_truth.py` + a recording.

## 5. Risks

- cuVSLAM availability/version on the laptop (needs the NVIDIA GPU enabled in
  WSL) and on GB10 (ARM, CUDA 13) — fallback: keep OpenVINS for M1–M2.
- Stereo depth quality on the net and flat walls (repeating texture aliasing
  seen in v33); map must mark these unknown, not free.
- Estimated wall textures in the sim; real venue appearance unverified.
- Faster takeoff/landing touches reviewed mission caps — needs team sign-off.

## 6. VLA note

Released OpenFly macro actions (3 m steps, 30° turns) are unsafe in ~1 m
corridors (3 m forward step collision-free from 2/60 route poses). A VLA could
at most act as a high-level selector over planner-verified options; it is not
on the critical path.
