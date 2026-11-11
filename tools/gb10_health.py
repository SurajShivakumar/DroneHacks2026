"""Read-only GB10 runtime health snapshot; no simulator or flight commands."""
import json
import os
from pathlib import Path
import subprocess
import time


def inspect(root=None, processes=None, project=None):
    root = Path(root or Path.home() / 'dronehacks-runtime/vio_ws')
    executable = root / 'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
    project = Path(project or Path.home() / 'dronehacks-2026-gb10')
    if processes is None:
        processes = subprocess.check_output(['ps', '-u', str(os.getuid()), '-o', 'pid=,args='], text=True)
    build = [p.strip() for p in processes.splitlines()
             if 'colcon build' in p and 'ov_msckf' in p]
    estimator = [p.strip() for p in processes.splitlines()
                 if '/run_subscribe_msckf' in p and 'ps ' not in p]
    log = root / 'log/latest_build/ov_msckf/stderr.log'
    errors = []
    if log.exists():
        errors = [line[:500] for line in log.read_text(errors='replace').splitlines()
                  if 'error:' in line.lower() or 'error ' in line.lower()][-8:]
    issues = []
    if not executable.exists() and not build:
        issues.append('vio_build_failed_or_missing')
    evidence = {}
    for label, pattern in (('sensors', 'gb10-sensor-smoke-*/result.json'),
                           ('vio', 'gb10-vio-shadow-*/result.json'),
                           ('diagnostic_rig', 'gb10-vio-rig-*/result.json'),
                           ('course_rig', 'gb10-course-rig-*/result.json'),
                           ('bootstrap', 'gb10-bootstrap-*/result.json'),
                           ('preflight', 'gb10-gcs-preflight-*/result.json'),
                           ('disarmed_offboard', 'gb10-disarmed-offboard-*/result.json'),
                           ('sky_beta_sensors', 'gb10-sky-beta-sensors-*/result.json'),
                           ('sky_beta_preflight', 'gb10-sky-beta-preflight-*/result.json'),
                           ('sky_beta_mission_readonly', 'gb10-sky-beta-mission-readonly-*/result.json'),
                           ('sky_beta_hover', 'gb10-sky-beta-hover-*/result.json'),
                           ('sky_beta_outbound', 'gb10-sky-beta-outbound*/result.json'),
                           ('sky_beta_astar', 'gb10-sky-beta-astar*/result.json'),
                           ('sky_beta_asset', 'gb10-sky-beta-passive-*/result.json')):
        files = sorted((project / 'artifacts').glob(pattern), key=lambda p: p.stat().st_mtime)
        if files:
            try:
                evidence[label] = {'path': str(files[-1]), **json.loads(files[-1].read_text())}
            except (ValueError, OSError):
                issues.append(label + '_evidence_unreadable')
    sensors=max((value for label,value in evidence.items() if label in ('sensors','sky_beta_sensors')),
                key=lambda value:Path(value['path']).stat().st_mtime,default={})
    if not sensors.get('sensor_streams_observed'):
        issues.append('sensor_evidence_missing_or_failed')
    # Diagnose the newest completed estimator run. Retain older failures above,
    # but do not repeatedly repair an old static test superseded by fresh evidence.
    candidates = [value for label, value in evidence.items() if label not in ('sensors','sky_beta_sensors','sky_beta_asset')]
    vio = max(candidates, key=lambda value: Path(value['path']).stat().st_mtime, default={})
    stats = vio.get('vio_odometry_observed', {})
    vio_observed = bool(stats.get('source_clock_observed_advancing')) and stats.get('messages', 0) >= 30
    if executable.exists() and not build and not estimator and not vio_observed:
        issues.append('vio_needs_shadow_validation')
    if vio.get('error') and not build:
        issues.append('latest_vio_test_failed')
    if vio.get('child_exit_codes', {}).get('openvins', 0) != 0:
        issues.append('diagnostic_rig_estimator_shutdown_failed')
    mission=max((value for label,value in evidence.items()
                 if label in ('sky_beta_hover','sky_beta_outbound','sky_beta_astar')),
                key=lambda value:Path(value['path']).stat().st_mtime,default={})
    if mission.get('mission_error'):
        issues.append('latest_requested_simulation_mission_failed_review_preserved_evidence')
    asset=evidence.get('sky_beta_asset',{})
    if asset.get('error') or asset.get('contact_solver_warning'):
        issues.append('latest_sky_beta_asset_check_failed')
    activity = list((root / 'build/ov_msckf').rglob('*.o')) if build else []
    activity += [p for p in (root / 'log/latest_build').rglob('*.log')] if build else []
    if build and activity and time.time() - max(p.stat().st_mtime for p in activity) > 900:
        issues.append('build_no_file_progress_for_15_minutes_inspect_before_intervening')
    return {'checked_unix': time.time(), 'issues': issues, 'build_processes': build,
            'estimator_processes': estimator, 'vio_binary_exists': executable.exists(),
            'latest_build_errors': errors, 'latest_test_evidence': evidence,
            'latest_estimator_evidence_path': vio.get('path'),
            'vio_advancing_observed_not_accuracy_qualified': vio_observed, 'flight_ready': False,
            'note': 'Process presence is not sensor/estimator qualification; inspect bounded run evidence.'}


if __name__ == '__main__':
    print(json.dumps(inspect(), indent=2))
