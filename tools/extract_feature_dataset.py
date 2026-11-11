"""Extract (truth pose, motion, OpenVINS SLAM feature count) samples from flown runs.

Offline evaluation data only: simulator truth here is used to *fit a planning
cost model* after flights, never as an input to flight code. Writes one CSV:
run,sim_s,x,y,z,yaw,vx,vy,yaw_rate,features,armed
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def truth(run):
    rows = []
    for line in open(run/'scene-poses.log'):
        try:
            r = json.loads(line)
        except Exception:
            continue
        h = r['header']['stamp']
        t = float(h.get('sec', 0))+float(h.get('nsec', 0))*1e-9
        for p in r['pose']:
            if p['name'] == 'sky_beta_estimated':
                q, s = p['orientation'], p['position']
                w, z = float(q.get('w', 1)), float(q.get('z', 0))
                x_, y_ = float(q.get('x', 0)), float(q.get('y', 0))
                yaw = np.arctan2(2*(w*z+x_*y_), 1-2*(y_*y_+z*z))
                rows.append((t, s.get('x', 0.), s.get('y', 0.), s.get('z', 0.), yaw))
                break
    a = np.array(sorted(rows))
    keep = np.r_[True, np.diff(a[:, 0]) > 0]
    return a[keep]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--artifacts', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--every-s', type=float, default=.2)
    a = p.parse_args()
    runs = sorted(d for d in a.artifacts.iterdir() if d.is_dir() and (d/'scene-poses.log').exists()
                  and (d/'vio-feature-clouds.jsonl').exists() and (d/'course-mission.json').exists())
    with a.output.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['run', 'sim_s', 'x', 'y', 'z', 'yaw', 'vx', 'vy', 'yaw_rate', 'features', 'armed'])
        for run in runs:
            try:
                m = json.loads((run/'course-mission.json').read_text())
                ev = {e['kind']: e for e in m.get('events', [])}
                if 'armed_observed' not in ev:
                    continue
                t0 = ev['armed_observed']['sim_s']
                t1 = ev.get('landed_disarmed', {}).get('disarmed_source_sim_s') or ev.get('request_land', {}).get('sim_s')
                if t1 is None:
                    continue
                T = truth(run)
                F = np.array([(r['source_s'], r['finite_xyz_count']) for r in map(json.loads, open(run/'vio-feature-clouds.jsonl'))
                              if r['topic'].endswith('slam')])
                yaw_u = np.unwrap(T[:, 4])
                n = 0
                for s in np.arange(t0-20, t1, a.every_s):
                    j = np.searchsorted(F[:, 0], s)
                    if j <= 0 or j >= len(F) or F[j, 0]-s > .2:
                        continue
                    k = np.searchsorted(T[:, 0], s)
                    if k < 5 or k >= len(T)-5:
                        continue
                    lo, hi = k-5, k+5
                    dt = T[hi, 0]-T[lo, 0]
                    if dt <= 0:
                        continue
                    vx, vy = (T[hi, 1]-T[lo, 1])/dt, (T[hi, 2]-T[lo, 2])/dt
                    yr = (yaw_u[hi]-yaw_u[lo])/dt
                    w.writerow([run.name, f'{s:.3f}', *(f'{v:.4f}' for v in T[k, 1:4]), f'{T[k, 4]:.4f}',
                                f'{vx:.4f}', f'{vy:.4f}', f'{yr:.4f}', int(F[j, 1]), int(s >= t0)])
                    n += 1
                print(run.name, n, flush=True)
            except Exception as exc:
                print(run.name, 'skipped', type(exc).__name__, exc, flush=True)


if __name__ == '__main__':
    main()
