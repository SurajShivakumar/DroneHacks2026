# v83: complete simulated course in 113.85 s (arm → disarm) — current fastest

Takeoff → all 9 ordered gates → pad landing/disarm. Native OpenVINS + PX4
Offboard, estimated Sky Beta, Gazebo Harmonic on GB10.

| Quantity | Value |
|---|---:|
| Arm → disarm (truth evaluation) | **113.85 s** |
| Takeoff / navigation / landing | 12.7 / 96.3 / 4.9 s |
| Course-structure contacts (5 ms, actual envelope) | **0** |
| Floor contact while navigating | none |
| Landing offset from pad centre | 5.6 cm, whole envelope inside ring |
| Tilt-aware gate audit | **9/9**, min lateral margin 6.3 cm |
| Max tilt | 2.82° |

## Pipeline ("best so far")

1. **Path**: published-map A* → Bezier corner curves (robust-corner route),
   10 cm smoothed; 8 cm mesh clearance screen + 9-gate audit pass.
2. **Headings**: copied exactly from the proven v51 reference (3/3 completions:
   146.4, 141.8, 144.7 s) — perception-aware heading DP on the published map.
3. **Speed**: `tools/prepare_fast_reference.py --heading-from k12 --baseline-ref k12
   --view-model artifacts/view-model-v2/model.pkl --min-features 30`: never slower
   than v51 at any point; faster only where the learned feature-count model
   (fit on 37 flights) predicts ≥ min(30, v51's prediction) OpenVINS features;
   image-flow cap 16 px/frame; soft 0.1 m/s passes at the 14 path kinks;
   v ≤ 0.9 m/s, a ≤ 0.25 m/s². Reference 85.5 s (v51: 121.7 s).
4. **Control**: supervisor `tools/course_flight.py` streams PX4 position +
   velocity + acceleration/yaw-rate feedforward; tracking clock slows only on
   cross-track error > 2.5 cm or along-track lag > 10 cm; 4 cm abort unchanged.
5. **Estimation**: OpenVINS stereo-inertial VIO → PX4 EKF2 (default config).

Settings `configs/sky_beta_course_speed_q1.json`; reference
`artifacts/published-course-speed-q1/`. Same-batch v84/v85 aborted at the maze
spot (-0.18, 1.10) — now a slow zone in `speed-r1`. Treat as n = 1 of 3.

## Recording

- `flight-video/flight.mp4` (118 s, source-timed, arm − 2 s → disarm + 2 s)
- `flight-video/flight-4x.mp4` (30 s)
Top: overhead observer (display only). Bottom: onboard stereo + OpenVINS tracks.
