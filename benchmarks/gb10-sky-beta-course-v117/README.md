# Sky Beta sim course run v117 — fm6 reference, 123.84 s arm→disarm, 9/9 gates

Simulated flight on GB10 (Gazebo Harmonic + PX4 v1.16 SITL + OpenVINS), 2026-11-07.
Full course, pad landing, no structure contact (truth evaluation in `evaluation/`).

## Pipeline

1. **Path**: published-map A* → Bezier corner curves (robust-corner route),
   10 cm smoothed; 8 cm mesh clearance screen + 9-gate audit pass.
2. **Headings**: the proven k12 perception-aware headings (`--heading-from`),
   including k12's 88° smoothstep start turn (11.6 s) and full stops at the
   14 path kinks. These are the parts the failure map says must not change:
   a steady-rate start turn (fm1/fm3) died in the start phase 5 of 10 times,
   and the corner-1 kink at s = 1.1 m kills VIO whenever a flight arrives
   there faster than k12 (fm5 v114).
3. **Speed**: `tools/prepare_fast_reference.py --speed-cap-table` with the
   failure-map v2 caps (60 flights, `tools/failure_map.py`): per 25 cm of
   route, the fastest speed a completed flight has survived there, 0.8× the
   slowest speed-related death within 0.5 m of any death, never below k12,
   and **identical to k12 for the first 2.5 m**. v ≤ 0.45 m/s, a ≤ 0.25 m/s²,
   yaw rate ≤ 0.25 rad/s. Reference 103.3 s (k12 121.7 s).
4. **Control**: supervisor `tools/course_flight.py`, PX4 position + velocity +
   acceleration/yaw-rate feedforward, tracking clock pauses on 2.5 cm
   cross-track or 10 cm lag; 4 cm abort unchanged.
5. **Estimation**: OpenVINS stereo-inertial VIO → PX4 EKF2. No simulator truth
   reaches flight code; truth is used only in the after-flight evaluation.

Reference `artifacts/published-course-learned-fm6/`, settings
`configs/sky_beta_course_learned_fm6.json`, launcher
`sim/runtime/gb10_slot_run.sh`. Guards unchanged (4 cm tracking abort / 2.5 cm
pause, VIO feature watchdog 8 features / 2.5 s, freshness, authority, reset and
preflight checks).

## fm5/fm6 family results (same headings, same first 2.5 m as k12)

| run | reference | result |
|---|---|---|
| v113 | fm5 | complete 125.0 s, 9/9 gates, contact-free |
| v114 | fm5 | VIO loss at the s = 1.1 m kink (fm5 arrived faster than k12 there; fm6 removes that) |
| v116 | fm6 | takeoff position/heading envelope fault (before the reference starts; 2 of 20 flights today) |
| v117 | fm6 | complete 123.8 s, 9/9 gates |
| v119–v121 | fm6 | see `artifacts/failure-map-v1/README.md` (results table) |

## Recording

- `flight-video/flight.mp4`: source-timed video, arm − 2 s to disarm + 2 s.
  Top: overhead observer (display aid only). Bottom: onboard stereo with
  OpenVINS feature tracks.
- Raw evidence (frames, VIO, PX4 ULog, mission journal) stays on GB10 under
  `~/dronehacks-2026-gb10/artifacts/gb10-sky-beta-course-v117/`.

## Caveats

- Estimated textured-wall scene (supplied OBJ/MTL is flat-coloured); the real
  venue appearance must be checked.
- The real drone flies cuVSLAM stereo-only through the organizer stack; see
  `artifacts/cuvslam-profile-comparison-v1/README.md` for why that changes the
  yaw-rate budget.
