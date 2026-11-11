# Experiment settings

These files record different experiments, not a ladder of automatically approved
flight presets. Higher version numbers and higher limits do not mean safer or
better. Preserve settings and source hashes with every run.

- `sky_beta_estimates.json`: explicit missing-propulsion and component estimates;
  not hardware measurements.
- `sky_beta_course_continuous_k12_ff.json`: conservative native OpenVINS baseline
  using acceleration/yaw feedforward.
- `sky_beta_course_fast_f5.json`: the earlier v55 speed experiment.
- `sky_beta_course_speed_q1.json`: v83's recorded profile. Its generated reference
  and fitted model live outside Git; successful completion was not repeatable in
  every same-batch trial.
- `sky_beta_course_learned_fm6.json`: v117's failure-map-based speed profile.
- `sky_beta_course_corridor_*.json`: corridor candidates; settings are not proof
  of a completed flight.
- `models/`: separate VLA dependency/runtime configurations.

Native benchmark settings do not override the organizer's flight-interface
contract or provide a hardware tuning file. Do not copy simulator PX4 parameters
to the real aircraft to make a test pass.
