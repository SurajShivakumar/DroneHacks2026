from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys

import pytest

from dronebench.benchmark_results import markdown_report, strategy_summary, validate_record, validate_records
from dronebench.experiments import STRATEGIES, digest


def artifacts(root):
    result = []
    for role, filename in (('code', 'code.py'), ('settings', 'settings.json'), ('evidence', 'events.json'), ('model', 'model.bin')):
        path = root / filename
        path.write_text(f'test fixture {role}')
        result.append({'role': role, 'path': filename, 'sha256': digest(path)})
    return result


def course(root):
    return {'schema_version': 1, 'run_id': 'fixture-course', 'strategy': 'classical',
            'kind': 'course_flight', 'status': 'success', 'provenance': artifacts(root),
            'clock_domain': 'simulation', 'flight_approved': True, 'approval_reference': 'test fixture only',
            'route': 'gate_b', 'wall_elapsed_s': 200., 'contacts': [{'stamp_sim_s': 14., 'evidence': 'events.json'}],
            'checkpoint_events': [{'event': event, 'stamp_sim_s': stamp, 'evidence': 'events.json', 'full_clearance': True}
                                  for event, stamp in zip(['start', 'corridor', 'maze_exit', 'tunnel', 'gate_b', 'landed_on_pad', 'disarmed'],
                                                         [10., 12., 14., 16., 18., 20., 21.])]}


def test_course_timing_comes_from_event_clock_not_wall_or_video(tmp_path):
    result = validate_record(course(tmp_path), tmp_path)
    assert result['finish_time_s'] == 11.
    assert result['penalty_adjusted_time_s'] == 16.
    assert result['wall_elapsed_s'] == 200.


def test_failed_run_keeps_checkpoint_but_has_no_finish(tmp_path):
    record = course(tmp_path)
    record.update(status='failed', reason='fixture estimator loss')
    record['checkpoint_events'] = record['checkpoint_events'][:3]
    result = validate_record(record, tmp_path)
    assert result['finish_time_s'] is None
    assert result['last_checkpoint_sim_s'] == 14.
    assert strategy_summary([result])[0]['course_attempts'] == 1
    assert strategy_summary([result])[0]['course_finishes'] == 0
    record['finish_time_s'] = 4.
    with pytest.raises(ValueError, match='only successful'):
        validate_record(record, tmp_path)


def test_unknown_contacts_are_represented_only_for_failed_flights(tmp_path):
    record=course(tmp_path)
    record.update(status='failed',reason='tracking abort; contact stream not recorded',contacts=None)
    result=validate_record(record,tmp_path)
    assert result['contacts'] is None and result['contact_observation_complete'] is False
    assert result['finish_time_s'] is None and result['penalty_adjusted_time_s'] is None
    record.update(status='success')
    with pytest.raises(ValueError,match='contacts'):
        validate_record(record,tmp_path)


@pytest.mark.parametrize('mutation', ['skip', 'regress', 'clearance', 'unapproved', 'fake_evidence', 'claimed_time'])
def test_invalid_completion_rejected(tmp_path, mutation):
    record = course(tmp_path)
    if mutation == 'skip':
        record['checkpoint_events'].pop(2)
    elif mutation == 'regress':
        record['checkpoint_events'][2]['stamp_sim_s'] = 11.
    elif mutation == 'clearance':
        record['checkpoint_events'][2]['full_clearance'] = False
    elif mutation == 'unapproved':
        record['flight_approved'] = False
    elif mutation == 'fake_evidence':
        record['checkpoint_events'][2]['evidence'] = 'nonexistent.json'
    elif mutation == 'claimed_time':
        record['finish_time_s'] = 3.
    with pytest.raises(ValueError):
        validate_record(record, tmp_path)


def test_diagnostic_span_and_inference_never_become_course_times(tmp_path):
    base = {'schema_version': 1, 'run_id': 'fixture-rig', 'strategy': 'classical',
            'kind': 'diagnostic_rig', 'status': 'success', 'provenance': artifacts(tmp_path),
            'clock_domain': 'simulation', 'sim_start_s': 3., 'sim_end_s': 23., 'wall_elapsed_s': 30.}
    rig = validate_record(base, tmp_path)
    assert rig['observation_span_sim_s'] == 20.
    assert rig['finish_time_s'] is None
    inference = deepcopy(base)
    inference.update(kind='offline_inference', strategy='openfly', run_id='fixture-inference', inference_latency_ms=[100., 300., 200.])
    result = validate_record(inference, tmp_path)
    assert result['inference_median_ms'] == 200.
    assert strategy_summary([rig, result])[1]['course_attempts'] == 0
    base['finish_time_s'] = 20.
    with pytest.raises(ValueError):
        validate_record(base, tmp_path)


def test_evidence_hashes_and_paths_verified(tmp_path):
    record = course(tmp_path)
    (tmp_path / 'events.json').write_text('modified')
    with pytest.raises(ValueError, match='hash mismatch'):
        validate_record(record, tmp_path)
    record['provenance'][0]['path'] = '../outside.py'
    with pytest.raises(ValueError, match='outside root'):
        validate_record(record, tmp_path)


def test_missing_strategies_and_blocked_models_explicit(tmp_path):
    record = {'schema_version': 1, 'run_id': 'fixture-blocked', 'strategy': 'openfly_v2',
              'kind': 'offline_inference', 'status': 'blocked', 'reason': 'fixture weights unavailable'}
    result = validate_record(record, tmp_path)
    report = markdown_report([result])
    assert 'blocked' in report and 'weights unavailable' in report
    assert 'NOT RUN' in report and 'aerovla' in report and 'world_model' in report
    assert len(strategy_summary([result])) == len(STRATEGIES)
    with pytest.raises(ValueError, match='duplicate'):
        validate_records([record, record], tmp_path)


def test_video_bytes_must_exist_and_match_claimed_hash(tmp_path):
    record = course(tmp_path)
    video = tmp_path / 'fixture.mp4'
    video.write_bytes(b'fixture bytes; decoding is separately validated')
    record['videos'] = [{'path': video.name, 'sha256': digest(video)}]
    result = validate_record(record, tmp_path)
    assert result['videos'][0]['verified_bytes'] is True
    video.write_bytes(b'tampered')
    with pytest.raises(ValueError, match='hash mismatch'):
        validate_record(record, tmp_path)
    record['videos'][0]['path'] = 'absent.mp4'
    with pytest.raises(ValueError, match='missing'):
        validate_record(record, tmp_path)


def test_proxy_hover_rollout_cannot_be_scored_as_course_finish(tmp_path):
    record = {'schema_version': 1, 'run_id': 'fixture-ppo-hover', 'strategy': 'teammate_ppo',
              'kind': 'proxy_rollout', 'status': 'success', 'provenance': artifacts(tmp_path),
              'sim_start_s': 0., 'sim_end_s': 7.04, 'clock_domain': 'simulation'}
    result = validate_record(record, tmp_path)
    assert result['observation_span_sim_s'] == 7.04
    assert result['finish_time_s'] is None
    record['finish_time_s'] = 7.04
    with pytest.raises(ValueError, match='only successful course flights'):
        validate_record(record, tmp_path)


def test_report_cli_preserves_input_and_exclusive_output(tmp_path):
    record = course(tmp_path)
    source = tmp_path / 'input.json'
    original = json.dumps({'schema_version': 1, 'runs': [record]})
    source.write_text(original)
    output = tmp_path / 'report'
    command = [sys.executable, 'tools/benchmark_report.py', str(source), str(output), '--root', str(tmp_path)]
    subprocess.run(command, check=True, capture_output=True)
    assert source.read_text() == original
    assert (output / 'input.json').read_text() == original
    assert '../events.json' in (output / 'report.md').read_text()
    assert subprocess.run(command, capture_output=True).returncode != 0
