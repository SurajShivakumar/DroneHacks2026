"""Replay a pinned teammate policy in its own simulator; no ROS/flight I/O."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np


def json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(type(value).__name__)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, required=True)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(args.repo.resolve()))
    import evaluate
    import torch
    torch.set_num_threads(2)
    result = {'kind': 'teammate_numpy_simulator_reproduction_not_px4_flight',
              'timing': 'airborne start to pad hover, excludes takeoff/landing/disarm',
              'files_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in (args.bundle/'model.zip', args.bundle/'vecnormalize.pkl',
                                         args.bundle/'policy.onnx', args.repo/'config/course_v3.yaml')}}
    for label, count, randomized in [('nominal', 1, False), ('randomized', 256, True)]:
        start = time.perf_counter()
        base, trajectories, outcomes = evaluate.rollout(str((args.bundle/'model.zip').resolve()),
                                                       n=count, randomize=randomized, seed=1)
        successful = [o['lap_time'] for o in outcomes if o['is_success']]
        result[label] = {'episodes': count, 'successes': len(successful), 'outcomes': outcomes,
                         'wall_s': time.perf_counter()-start,
                         'best_hover_time_s': min(successful) if successful else None,
                         'median_hover_time_s': float(np.median(successful)) if successful else None,
                         'worst_hover_time_s': max(successful) if successful else None}
        for index, trajectory in enumerate(trajectories):
            np.savetxt(args.output/f'{label}-{index:03d}.csv', trajectory, delimiter=',',
                       header='t,x,y,z,vx,vy,vz,qw,qx,qy,qz,a_thrust,a_roll_rate,a_pitch_rate,a_yaw_rate', comments='')
        print(label, {k: v for k, v in result[label].items() if k != 'outcomes'}, flush=True)
        (args.output/'result.json').write_text(json.dumps(result, indent=2, default=json_value), encoding='utf-8')
    spec = importlib.util.spec_from_file_location('teammate_deploy_test', args.repo/'tests/test_deploy_loop.py')
    deploy_test = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(deploy_test)
    result['onnx_frame_loop'] = []
    for heading in (None, 0., 2.3):
        ok, duration, detail = deploy_test.run(str((args.bundle/'policy.onnx').resolve()), heading=heading)
        result['onnx_frame_loop'].append({'heading': heading, 'success': bool(ok),
                                          'hover_time_s': duration, 'detail': str(detail)})
    (args.output/'result.json').write_text(json.dumps(result, indent=2, default=json_value), encoding='utf-8')
    print(json.dumps(result['onnx_frame_loop']), flush=True)


if __name__ == '__main__':
    main()
