"""Open-loop VLA route agreement on recorded onboard frames (no flight commands).

Stage `prepare` (ROS/system python, needs the teammate course repo):
  samples recorded onboard frames from a completed flight, labels the expert's
  next macro move from simulator truth *after the flight* (evaluation only), and
  pre-computes whether each OpenFly macro action would be collision-free from
  that truth pose against the published course geometry.
Stage `infer` (models venv, GPU): runs pinned OpenFly-Agent7B (`vlnv1`) on the
  same three-frame histories and decodes actions with the upstream eval rule
  (round, exact match to the 10 macro vectors, else 0/stop).
Stage `score`: agreement and would-be-safe rates per expert class.

Truth is used only to grade a recorded flight, never to command anything.
"""
import argparse
from hashlib import sha256
import json
from math import pi
from pathlib import Path
import sys
import time

import numpy as np

ACTIONS = ['stop', 'forward3', 'turn_left30', 'turn_right30', 'up3', 'down3',
           'left3', 'right3', 'forward6', 'forward9']
# Upstream train/eval.py convert_to_action_id vectors (commit c0750754).
ACTION_VECTORS = [[1, 0, 0, 0, 0, 0, 0, 0], [0, 3, 0, 0, 0, 0, 0, 0], [0, 0, 15, 0, 0, 0, 0, 0],
                  [0, 0, 0, 15, 0, 0, 0, 0], [0, 0, 0, 0, 2, 0, 0, 0], [0, 0, 0, 0, 0, 2, 0, 0],
                  [0, 0, 0, 0, 0, 0, 5, 0], [0, 0, 0, 0, 0, 0, 0, 5], [0, 6, 0, 0, 0, 0, 0, 0],
                  [0, 9, 0, 0, 0, 0, 0, 0]]
# Expert class each action counts as agreeing with.
ACTION_CLASS = {'stop': 'stop', 'forward3': 'forward', 'forward6': 'forward', 'forward9': 'forward',
                'turn_left30': 'turn_left', 'turn_right30': 'turn_right', 'up3': 'up', 'down3': 'down',
                'left3': 'left', 'right3': 'right'}
INSTRUCTION = ('Fly through the indoor maze below the walls: follow the corridor around each turn, '
               'pass through the tube and the window, then land on the landing pad.')
WALL_TOP_M = 1.727


def yaw_of(q):
    w, x, y, z = q.T
    return np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def expert_label(t, p, yaw, i, end_i, move_m=.25, turn_rad=np.deg2rad(15)):
    """First of: |yaw change| > 15 deg, or displacement > 0.25 m (body frame)."""
    for j in range(i+1, end_i):
        dyaw = (yaw[j]-yaw[i]+pi) % (2*pi)-pi
        d = p[j]-p[i]
        if abs(dyaw) > turn_rad and np.linalg.norm(d) < move_m:
            return ('turn_left' if dyaw > 0 else 'turn_right'), float(t[j]-t[i])
        if np.linalg.norm(d) >= move_m:
            c, s = np.cos(yaw[i]), np.sin(yaw[i])
            body = np.array([c*d[0]+s*d[1], -s*d[0]+c*d[1], d[2]])
            k = int(np.argmax(np.abs(body)))
            name = [('forward', 'backward'), ('left', 'right'), ('up', 'down')][k][0 if body[k] > 0 else 1]
            return name, float(t[j]-t[i])
    return 'stop', None


def action_safety(geometry, pos, yaw, radius, half_height):
    """Collision-free check of each macro from a truth pose (published boxes/net/floor)."""
    c, s = np.cos(yaw), np.sin(yaw)
    fwd, left, up = np.array([c, s, 0.]), np.array([-s, c, 0.]), np.array([0., 0., 1.])
    out = {}
    for name in ACTIONS:
        if name in ('stop', 'turn_left30', 'turn_right30'):
            out[name] = {'collision_free': True, 'leaves_below_wall_volume': False}
            continue
        step = {'forward3': (fwd, 3.), 'forward6': (fwd, 6.), 'forward9': (fwd, 9.), 'up3': (up, 3.),
                'down3': (-up, 3.), 'left3': (left, 3.), 'right3': (-left, 3.)}[name]
        direction, length = step
        side = np.cross(direction, up) if abs(direction[2]) < .5 else fwd
        side /= np.linalg.norm(side)
        normal2 = np.cross(direction, side)
        nearest = np.inf
        for a in np.linspace(-1, 1, 5):
            for b in np.linspace(-1, 1, 3):
                origin = pos + a*radius*side + b*(half_height if abs(direction[2]) < .5 else radius)*normal2
                hits = geometry.cast(origin, direction[None, :])
                if hits.origin_inside_solid:
                    nearest = 0.
                elif np.isfinite(hits.t[0]):
                    nearest = min(nearest, float(hits.t[0]))
        reach = length + (radius if abs(direction[2]) < .5 else half_height)
        end_z = pos[2]+direction[2]*length
        out[name] = {'collision_free': bool(nearest > reach), 'nearest_hit_m': None if not np.isfinite(nearest) else nearest,
                     'leaves_below_wall_volume': bool(end_z+half_height > WALL_TOP_M or end_z-half_height < 0)}
    return out


def prepare(a):
    sys.path.insert(0, str(a.repo.resolve()))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from drl.config import load_config
    from drl.course import load_obj_groups
    from dronebench.published_raycast import build_published_raycast
    config, course, _ = load_config(str(a.repo.resolve()/'config/course_v3.yaml'))
    obj = a.repo/config['course_obj']
    _, groups = load_obj_groups(str(obj))
    net = np.concatenate([pts for name, pts in groups.items() if name.startswith('Net')])
    bounds = tuple(v for axis in range(3) for v in (net[:, axis].min(), net[:, axis].max()))
    geometry = build_published_raycast(course, published_provenance='published OBJ:'+sha256(obj.read_bytes()).hexdigest(),
                                       net_bounds=bounds, floor_z=0.)
    contract = json.loads(a.contract.read_text())
    radius, half_height = float(contract['collision_radius_m']), float(contract['collision_half_height_m'])
    truth = np.loadtxt(a.truth, delimiter=',', skiprows=1)
    t, p, yaw = truth[:, 0], truth[:, 1:4], yaw_of(truth[:, 7:11])
    manifest_path = next(a.frames.glob('*.json'))
    manifest = json.loads(manifest_path.read_text())
    frames = [f for f in manifest['frames'] if a.start_s <= f['stamp_ns']/1e9 <= a.end_s]
    stamps = np.array([f['stamp_ns']/1e9 for f in frames])
    picks = np.linspace(a.start_s+1.5, a.end_s-.5, a.samples)
    end_i = int(np.searchsorted(t, a.end_s))
    rows = []
    for target in picks:
        history = []
        for back in (1.0, .5, 0.):
            k = int(np.argmin(np.abs(stamps-(target-back))))
            history.append(frames[k])
        if len({f['stamp_ns'] for f in history}) < 3:
            continue
        now = history[-1]['stamp_ns']/1e9
        i = int(np.searchsorted(t, now))
        label, lead = expert_label(t, p, yaw, i, end_i)
        rows.append({'sim_s': now, 'truth_position_m': p[i].tolist(), 'truth_yaw_rad': float(yaw[i]),
                     'expert': label, 'expert_lead_s': lead,
                     'front_history': [{'path': str((a.frames/f['file']).resolve()), 'stamp_ns': int(f['stamp_ns']),
                                        'sha256': sha256((a.frames/f['file']).read_bytes()).hexdigest()} for f in history],
                     'action_safety': action_safety(geometry, p[i], yaw[i], radius, half_height)})
    out = {'kind': 'open_loop_vla_route_agreement_inputs_not_flight', 'instruction': INSTRUCTION,
           'frames_manifest': str(manifest_path), 'frames_manifest_sha256': sha256(manifest_path.read_bytes()).hexdigest(),
           'frames_topic': manifest.get('topic'), 'truth_sha256': sha256(a.truth.read_bytes()).hexdigest(),
           'interval_sim_s': [a.start_s, a.end_s], 'envelope': {'radius_m': radius, 'half_height_m': half_height},
           'geometry_omits': list(geometry.omitted_geometry), 'samples': rows}
    a.output.write_text(json.dumps(out, indent=1))
    from collections import Counter
    print(len(rows), Counter(r['expert'] for r in rows))


def infer(a):
    import torch
    from PIL import Image
    from transformers import AutoConfig, AutoImageProcessor, AutoModelForVision2Seq, AutoProcessor
    data = json.loads(a.inputs.read_text())
    sys.path.insert(0, str(a.upstream/'train'))
    from extern.hf.configuration_prismatic import OpenFlyConfig
    from extern.hf.modeling_prismatic import OpenVLAForActionPrediction
    from extern.hf.processing_prismatic import PrismaticImageProcessor, PrismaticProcessor
    AutoConfig.register('openvla', OpenFlyConfig)
    AutoImageProcessor.register(OpenFlyConfig, PrismaticImageProcessor)
    AutoProcessor.register(OpenFlyConfig, PrismaticProcessor)
    AutoModelForVision2Seq.register(OpenFlyConfig, OpenVLAForActionPrediction)
    processor = AutoProcessor.from_pretrained(str(a.checkpoint))
    import timm
    original = timm.create_model
    timm.create_model = lambda *v, **o: original(*v, **{**o, 'pretrained': False})
    try:
        model, loading = AutoModelForVision2Seq.from_pretrained(
            str(a.checkpoint), torch_dtype=torch.bfloat16, low_cpu_mem_usage=True,
            attn_implementation='sdpa', output_loading_info=True)
    finally:
        timm.create_model = original
    if any(loading.get(k) for k in ('missing_keys', 'unexpected_keys', 'mismatched_keys', 'error_msgs')):
        raise RuntimeError('checkpoint does not fully cover the model')
    model = model.to('cuda').eval()
    instruction = a.instruction or data['instruction']
    results = []
    with torch.inference_mode():
        for n, row in enumerate(data['samples']):
            for f in row['front_history']:
                if sha256(Path(f['path']).read_bytes()).hexdigest() != f['sha256']:
                    raise ValueError('frame hash mismatch')
            images = [Image.open(f['path']).convert('RGB') for f in row['front_history']]
            torch.cuda.synchronize(); t0 = time.perf_counter()
            inputs = processor(instruction, images).to('cuda', dtype=torch.bfloat16)
            raw = model.predict_action(**inputs, unnorm_key='vlnv1', do_sample=False)
            torch.cuda.synchronize(); dt = time.perf_counter()-t0
            rounded = np.asarray(raw).round().astype(int).tolist()
            action_id = next((k for k, v in enumerate(ACTION_VECTORS) if v == rounded), 0)
            results.append({'sim_s': row['sim_s'], 'raw': np.asarray(raw).tolist(), 'action_id': action_id,
                            'action': ACTIONS[action_id], 'exact_match': rounded in ACTION_VECTORS, 'wall_s': dt})
            print(n, row['expert'], ACTIONS[action_id], f'{dt:.3f}s', flush=True)
    a.output.write_text(json.dumps({'kind': 'open_loop_vla_predictions_not_flight', 'model': 'OpenFly-Agent7B bf16',
                                    'unnorm_key': 'vlnv1', 'instruction': instruction,
                                    'inputs_sha256': sha256(a.inputs.read_bytes()).hexdigest(),
                                    'gpu': torch.cuda.get_device_name(), 'predictions': results}, indent=1))


def score(a):
    data = json.loads(a.inputs.read_text())
    preds = json.loads(a.predictions.read_text())['predictions']
    by_s = {round(p['sim_s'], 6): p for p in preds}
    from collections import Counter, defaultdict
    table = defaultdict(Counter)
    agree = safe = total = 0
    expert_safe = 0
    for row in data['samples']:
        p = by_s.get(round(row['sim_s'], 6))
        if p is None:
            continue
        total += 1
        table[row['expert']][p['action']] += 1
        agree += ACTION_CLASS[p['action']] == row['expert']
        s = row['action_safety'][p['action']]
        safe += s['collision_free'] and not s['leaves_below_wall_volume']
    latency = sorted(p['wall_s'] for p in preds)
    summary = {'kind': 'open_loop_vla_route_agreement_not_flight', 'samples': total,
               'agreement_rate': agree/total if total else None,
               'predicted_action_collision_free_rate': safe/total if total else None,
               'prediction_histogram': dict(Counter(p['action'] for p in preds)),
               'expert_histogram': dict(Counter(r['expert'] for r in data['samples'])),
               'confusion_expert_to_predicted': {k: dict(v) for k, v in table.items()},
               'safe_forward3_rate_from_truth_poses': sum(r['action_safety']['forward3']['collision_free'] for r in data['samples'])/len(data['samples']),
               'median_inference_wall_s': latency[len(latency)//2] if latency else None,
               'limitations': ('Open-loop on frames from a planner-flown trajectory; one instruction; upstream '
                               'macro scale (3 m steps, 30 deg turns) is intrinsic to the released model. Geometry '
                               'check omits the tube shell and boulders, so collision-free rates are optimistic.')}
    a.output.write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary, indent=1))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='cmd', required=True)
    q = sub.add_parser('prepare')
    q.add_argument('--repo', type=Path, required=True); q.add_argument('--frames', type=Path, required=True)
    q.add_argument('--truth', type=Path, required=True); q.add_argument('--contract', type=Path, required=True)
    q.add_argument('--start-s', type=float, required=True); q.add_argument('--end-s', type=float, required=True)
    q.add_argument('--samples', type=int, default=150); q.add_argument('--output', type=Path, required=True)
    q = sub.add_parser('infer')
    q.add_argument('--inputs', type=Path, required=True); q.add_argument('--upstream', type=Path, required=True)
    q.add_argument('--checkpoint', type=Path, required=True); q.add_argument('--instruction')
    q.add_argument('--output', type=Path, required=True)
    q = sub.add_parser('score')
    q.add_argument('--inputs', type=Path, required=True); q.add_argument('--predictions', type=Path, required=True)
    q.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    {'prepare': prepare, 'infer': infer, 'score': score}[a.cmd](a)


if __name__ == '__main__':
    main()
