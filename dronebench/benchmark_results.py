"""Validate measured benchmark records without turning previews into flight scores.

This checks records and hashed evidence, not physical checkpoint detection or
video decoding. An independent evaluator must produce the underlying events.
"""

from copy import deepcopy
from math import isfinite
from pathlib import Path
from statistics import median

from dronebench.experiments import STRATEGIES, digest


KINDS = {'course_flight', 'diagnostic_rig', 'offline_inference', 'component', 'proxy_rollout'}
STATUSES = {'success', 'failed', 'blocked', 'not_run'}


def number(value, name):
    if type(value) not in (int, float) or not isfinite(value) or value < 0:
        raise ValueError(f'{name} must be finite and nonnegative')
    return value


def verify_artifact(item, root):
    """Resolve only evidence inside root; verify bytes, never execute them."""
    if not isinstance(item, dict) or not isinstance(item.get('path'), str):
        raise ValueError('artifact path required')
    relative = Path(item['path'])
    if relative.is_absolute():
        raise ValueError('artifact paths must be relative to evidence root')
    path = (root / relative).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError(f'artifact missing or outside root: {relative}')
    if digest(path) != item.get('sha256'):
        raise ValueError(f'artifact hash mismatch: {relative}')
    return dict(item, path=relative.as_posix(), verified_bytes=True)


def validate_record(record, root):
    root = Path(root).resolve()
    result = deepcopy(record)
    if record.get('schema_version') != 1:
        raise ValueError('schema_version must be 1')
    if not isinstance(record.get('run_id'), str) or not record['run_id'].strip():
        raise ValueError('run_id required')
    if record.get('strategy') not in STRATEGIES or record.get('kind') not in KINDS:
        raise ValueError('unknown strategy or benchmark kind')
    if record.get('status') not in STATUSES:
        raise ValueError('unknown outcome')
    executed = record['status'] in ('success', 'failed')
    if not executed and any(record.get(key) is not None for key in ('wall_elapsed_s', 'sim_start_s', 'sim_end_s', 'finish_time_s')):
        raise ValueError('unexecuted records cannot contain measured timings')
    if record['status'] != 'success' and not str(record.get('reason') or '').strip():
        raise ValueError('failed, blocked and not_run records require a reason')
    provenance = record.get('provenance', [])
    result['provenance'] = [verify_artifact(item, root) for item in provenance]
    if executed:
        roles = {item.get('role') for item in provenance}
        if not {'code', 'settings', 'evidence'} <= roles:
            raise ValueError('executed records require hashed code, settings and evidence')
    result['videos'] = [verify_artifact(item, root) for item in record.get('videos', [])]
    if any(Path(item['path']).suffix.lower() not in ('.mp4', '.webm', '.mkv') for item in result['videos']):
        raise ValueError('video evidence must name a video container')
    if record.get('finish_time_s') is not None and (record['kind'] != 'course_flight' or record['status'] != 'success'):
        raise ValueError('only successful course flights may have a finish time')
    result['finish_time_s'] = None
    result['penalty_adjusted_time_s'] = None
    result['last_checkpoint_sim_s'] = None
    result['observation_span_sim_s'] = None
    result['inference_median_ms'] = None
    if record.get('wall_elapsed_s') is not None:
        number(record['wall_elapsed_s'], 'wall_elapsed_s')
    if record.get('sim_start_s') is not None or record.get('sim_end_s') is not None:
        start = number(record.get('sim_start_s'), 'sim_start_s')
        end = number(record.get('sim_end_s'), 'sim_end_s')
        if end < start:
            raise ValueError('simulation time regressed')
        if record.get('clock_domain') != 'simulation':
            raise ValueError('simulation timestamps require explicit simulation clock domain')
        result['observation_span_sim_s'] = end - start
    if record['kind'] == 'offline_inference' and executed:
        samples = record.get('inference_latency_ms', [])
        if record['status'] == 'success' and not samples:
            raise ValueError('successful inference requires measured latency samples')
        if any(number(sample, 'latency') <= 0 for sample in samples):
            raise ValueError('latency samples must be positive')
        if samples:
            result['inference_median_ms'] = median(samples)
        if 'model' not in {item.get('role') for item in provenance}:
            raise ValueError('inference requires hashed model provenance')
    if record['kind'] == 'course_flight' and executed:
        if record.get('clock_domain') != 'simulation':
            raise ValueError('course timing requires simulation clock')
        if record.get('flight_approved') is not True or not record.get('approval_reference'):
            raise ValueError('executed flight requires recorded team approval')
        route = record.get('route')
        if route not in ('gate_b', 'window'):
            raise ValueError('explicit route required')
        expected = ['start', 'corridor', 'maze_exit', 'tunnel', route, 'landed_on_pad', 'disarmed']
        events = record.get('checkpoint_events', [])
        evidence_paths = {item['path'] for item in provenance if item.get('role') == 'evidence'}
        previous = -1
        for index, event in enumerate(events):
            if index >= len(expected) or event.get('event') != expected[index]:
                raise ValueError('checkpoint order mismatch')
            stamp = number(event.get('stamp_sim_s'), 'event stamp')
            if stamp <= previous or event.get('evidence') not in evidence_paths:
                raise ValueError('events need advancing timestamps and evidence references')
            if index > 0 and index < 6 and event.get('full_clearance') is not True:
                raise ValueError('whole-aircraft checkpoint clearance required')
            previous = stamp
        if events:
            checkpoints = [e for e in events if e['event'] not in ('start', 'disarmed')]
            result['last_checkpoint_sim_s'] = checkpoints[-1]['stamp_sim_s'] if checkpoints else None
        contacts = record.get('contacts')
        unknown_failed_contacts=(record['status']=='failed' and 'contacts' in record and contacts is None)
        if not isinstance(contacts, list) and not unknown_failed_contacts:
            raise ValueError('contacts must be an explicit observed list')
        # An incomplete failed flight can honestly lack contact observations.
        # Keep null in its result and never derive penalties or a finish. A
        # successful flight still requires the original observed-contact list.
        result['contact_observation_complete']=not unknown_failed_contacts
        for contact in contacts or []:
            number(contact.get('stamp_sim_s'), 'contact stamp')
            if contact.get('evidence') not in evidence_paths:
                raise ValueError('contact evidence required')
        if record['status'] == 'success':
            if len(events) != len(expected):
                raise ValueError('successful course flight needs every checkpoint, landing and disarm')
            elapsed = events[-1]['stamp_sim_s'] - events[0]['stamp_sim_s']
            if record.get('finish_time_s') is not None and abs(number(record['finish_time_s'], 'finish_time_s') - elapsed) > 1e-8:
                raise ValueError('claimed finish time differs from measured events')
            result['finish_time_s'] = elapsed
            result['timing_definition'] = 'recorded start event to disarmed on pad; supplied rule 7.5 stops at disarm'
            count = sum(events[0]['stamp_sim_s'] <= c['stamp_sim_s'] <= events[-1]['stamp_sim_s'] for c in contacts)
            result['penalty_adjusted_time_s'] = elapsed + 5 * count
    return result


def validate_records(records, root):
    ids = [item.get('run_id') for item in records]
    if len(ids) != len(set(ids)):
        raise ValueError('duplicate run_id; preserve separate trials with unique IDs')
    return [validate_record(item, root) for item in records]


def strategy_summary(records):
    summary = []
    for name in STRATEGIES:
        rows = [r for r in records if r['strategy'] == name]
        flights = [r for r in rows if r['kind'] == 'course_flight' and r['status'] in ('success', 'failed')]
        finished = [r for r in flights if r['finish_time_s'] is not None]
        summary.append({'strategy': name, 'course_attempts': len(flights), 'course_finishes': len(finished),
                        'best_finish_sim_s': min((r['finish_time_s'] for r in finished), default=None),
                        'median_finish_sim_s': median([r['finish_time_s'] for r in finished]) if finished else None,
                        'median_penalty_adjusted_sim_s': median([r['penalty_adjusted_time_s'] for r in finished]) if finished else None,
                        'records': rows})
    return summary


def markdown_report(records):
    def fmt(value):
        return 'unavailable' if value is None else f'{value:.3f}'

    def cell(value):
        return str(value).replace('|', '\\|').replace('\n', ' ')

    lines = ['# Measured strategy benchmarks', '',
             'Simulation finish times, wall time and inference latency are separate. '
             'Videos and diagnostic rigs do not establish course completion. '
             'Successful course timing runs from the recorded start event to disarm on the pad, following supplied rule 7.5. '
             'Contact adjustment uses the supplied five-second rule and is not an official grade. '
             'Aggregates pool the supplied records; compare scene, route, settings and host before interpreting them. '
             'Evidence hashes are checked; video decoding and event correctness require independent validation.', '',
             '| Strategy | Course finishes / attempts | Best finish (sim s) | Median finish (sim s) | Median with contact penalties (sim s) |',
             '| --- | --- | --- | --- | --- |']
    for item in strategy_summary(records):
        lines.append(f"| {item['strategy']} | {item['course_finishes']} / {item['course_attempts']} | "
                     f"{fmt(item['best_finish_sim_s'])} | {fmt(item['median_finish_sim_s'])} | {fmt(item['median_penalty_adjusted_sim_s'])} |")
    lines += ['', '| Run | Strategy | Kind | Outcome | Wall s | Observed sim span s | Inference median ms | Evidence / video | Reason |',
              '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for row in records:
        links = [f"[{cell(Path(a['path']).name)}](<{a['path']}>)" for a in row['videos']]
        links += [f"[{cell(Path(a['path']).name)}](<{a['path']}>)" for a in row['provenance'] if a.get('role') == 'evidence']
        lines.append(f"| {cell(row['run_id'])} | {row['strategy']} | {row['kind']} | {row['status']} | "
                     f"{fmt(row.get('wall_elapsed_s'))} | {fmt(row['observation_span_sim_s'])} | "
                     f"{fmt(row['inference_median_ms'])} | {', '.join(links) or 'unavailable'} | {cell(row.get('reason', ''))} |")
    for name in STRATEGIES:
        if not any(row['strategy'] == name for row in records):
            lines.append(f'| unavailable | {name} | unavailable | NOT RUN | unavailable | unavailable | unavailable | unavailable | No measured record supplied |')
    return '\n'.join(lines) + '\n'
