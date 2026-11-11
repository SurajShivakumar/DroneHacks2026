import json
from pathlib import Path
import pytest

from tools.gb10_health import inspect


def sensor_evidence(project):
    path = project / 'artifacts/gb10-sensor-smoke-v1/result.json'
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps({'sensor_streams_observed': True}))


def test_failed_build_identified(tmp_path):
    root = tmp_path / 'runtime'
    log = root / 'log/latest_build/ov_msckf/stderr.log'
    log.parent.mkdir(parents=True)
    log.write_text('fatal error: removed header\n')
    result = inspect(root, '', tmp_path / 'project')
    assert 'vio_build_failed_or_missing' in result['issues']
    assert result['latest_build_errors'] == ['fatal error: removed header']


def test_progressing_build_is_not_missing_failure(tmp_path):
    sensor_evidence(tmp_path / 'project')
    result = inspect(tmp_path / 'runtime', '12 colcon build --packages-select ov_msckf', tmp_path / 'project')
    assert not result['issues']


def test_executable_alone_does_not_prove_vio(tmp_path):
    sensor_evidence(tmp_path / 'project')
    root = tmp_path / 'runtime'
    exe = root / 'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
    exe.parent.mkdir(parents=True)
    exe.touch()
    result = inspect(root, '', tmp_path / 'project')
    assert result['issues'] == ['vio_needs_shadow_validation']
    assert not result['flight_ready']


def test_new_course_run_supersedes_failed_static_test(tmp_path):
    import os
    project = tmp_path / 'project'
    sensor_evidence(project)
    root = tmp_path / 'runtime'
    exe = root / 'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf'
    exe.parent.mkdir(parents=True)
    exe.touch()
    old = project / 'artifacts/gb10-vio-shadow-v1/result.json'
    old.parent.mkdir(parents=True)
    old.write_text(json.dumps({'error': 'static initialization incomplete'}))
    os.utime(old, (1, 1))
    fresh = project / 'artifacts/gb10-gcs-preflight-v2/result.json'
    fresh.parent.mkdir(parents=True)
    fresh.write_text(json.dumps({'vio_odometry_observed': {
        'source_clock_observed_advancing': True, 'messages': 100},
        'child_exit_codes': {'openvins': 0}}))
    result = inspect(root, '', project)
    assert not result['issues']
    assert result['latest_test_evidence']['vio']['error']
    assert result['latest_estimator_evidence_path'] == str(fresh)
    assert not result['flight_ready']


@pytest.mark.parametrize('run',['gb10-sky-beta-outbound-net-v2','gb10-sky-beta-astar-net-v1'])
def test_failed_requested_course_is_not_hidden_by_advancing_vio(tmp_path,run):
    project=tmp_path/'project';sensor_evidence(project)
    root=tmp_path/'runtime'
    exe=root/'install/ov_msckf/lib/ov_msckf/run_subscribe_msckf';exe.parent.mkdir(parents=True);exe.touch()
    report=project/'artifacts'/run/'result.json';report.parent.mkdir(parents=True)
    report.write_text(json.dumps({'vio_odometry_observed':{'source_clock_observed_advancing':True,'messages':100},
        'mission_error':'tracking exceeded screened margin','all_children_stopped':True}))
    result=inspect(root,'',project)
    assert result['issues']==['latest_requested_simulation_mission_failed_review_preserved_evidence']
    assert not result['flight_ready']
