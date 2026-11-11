# Tools

Run from the repository root. Names alone do not indicate flight qualification.
Read the selected tool's docstring and `--help`; tools that publish controls need
a reviewed mission and fresh telemetry. Offline tools do not authorize flight.

| Group | Main entry points |
|---|---|
| Published-map path | `plan_published_course.py`, `prepare_robust_corner_reference.py`, `plan_corridor_path.py` |
| Motion and camera heading | `prepare_fast_reference.py`, `prepare_heading_reference.py`, `prepare_efficient_reference.py` |
| Route/aircraft audit | `check_sky_beta_reference.py`, `check_whole_aircraft_gates.py`, `audit_scene_gates.py` |
| Native simulator flight | `course_flight.py`, `hover_qualification.py` (native `/fmu/in/*`, not organizer-facing deployment) |
| Organizer-facing adapters | `skybeta_team_nav.py`, `skybeta_pathplan_launch.py`; preserve `/skybeta/*` ownership |
| Estimation integration | `ev_bridge.py`, `cuvslam_vo_node.py`, `skybeta_vio_relay.py` |
| Read-only/offline estimation diagnosis | `observe_published_stereo.py`, `review_*`, `benchmark_cuvslam.py`, `eval_estimator_replay.py` |
| Independent post-flight evaluation | `evaluate_flight_truth.py`, `tracking_diagnostics.py`, `failure_map.py` |
| Recording and summaries | `encode_flight_video.py`, `encode_recording.py`, `benchmark_report.py` |
| Offline model research | `model_benchmark.py`, `stage_model.py`, `vla_route_agreement.py` |
| Asset preparation | `build_course.py`, `import_sky_beta_cad.py` |

Truth-evaluation tools are deliberately separate from navigation. Many historical
experiment commands require ignored assets/outputs; see
[reproduction notes](../docs/REPRODUCING.md). Local scheduler/agent-maintenance
scripts are excluded from the source release and no checker is installed by it.
