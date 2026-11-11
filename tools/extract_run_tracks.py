"""Per-run summary for the failure map (offline evaluation data, run on GB10).

For every run folder with course-mission.json + scene-poses.log, write one JSON
line: outcome, reason, event times, gates, reference file, estimator, the truth
track sampled every 0.1 s between arm and disarm (t,x,y,z,yaw) and the minimum
SLAM feature count per 0.5 s. Simulator truth is used only to grade flights
after the fact; nothing here feeds flight code.
"""
import argparse
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
                rows.append((t, s.get('x', 0.), s.get('y', 0.), s.get('z', 0.),
                             np.arctan2(2*(w*z+x_*y_), 1-2*(y_*y_+z*z))))
                break
    a = np.array(sorted(rows))
    return a[np.r_[True, np.diff(a[:, 0]) > 0]]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--artifacts', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    out = a.output.open('w')
    for run in sorted(a.artifacts.iterdir()):
        if not (run/'course-mission.json').exists() or not (run/'scene-poses.log').exists():
            continue
        try:
            m = json.loads((run/'course-mission.json').read_text())
            ev = {e['kind']: e for e in m.get('events', [])}
            if 'armed_observed' not in ev:
                continue
            t0 = ev['armed_observed']['sim_s']
            t1 = (ev.get('landed_disarmed', {}).get('disarmed_source_sim_s')
                  or ev.get('request_land', {}).get('sim_s') or ev.get('navigation_abort', {}).get('sim_s'))
            if t1 is None:
                continue
            T = truth(run)
            grid = np.arange(t0, t1, .1)
            track = np.column_stack([grid]+[np.interp(grid, T[:, 0], T[:, i]) for i in (1, 2, 3)]
                                    + [np.interp(grid, T[:, 0], np.unwrap(T[:, 4]))])
            feats = []
            fpath = run/'vio-feature-clouds.jsonl'
            if fpath.exists():
                F = np.array([(r['source_s'], r['finite_xyz_count']) for r in map(json.loads, open(fpath))
                              if r['topic'].endswith('slam')])
                for s in np.arange(t0, t1, .5):
                    k = (F[:, 0] >= s) & (F[:, 0] < s+.5)
                    feats.append([round(s, 1), int(F[k, 1].min()) if k.any() else None])
            res = {}
            if (run/'result.json').exists():
                r = json.loads((run/'result.json').read_text())
                res = {'estimator': r.get('estimator', 'openvins'),
                       'trajectory': next((k for k in r.get('mission_asset_sha256', {}) if 'trajectory' in k), None)}
            traj = None
            for cand in (run/'settings').glob('reference-*'):
                traj = cand.name
            rec = {'run': run.name, 'state': m.get('state'), 'reason': m.get('reason'),
                   'arm_s': t0, 'nav_s': ev.get('navigation_started', {}).get('sim_s'),
                   'abort_s': ev.get('navigation_abort', ev.get('vio_quality_abort', {})).get('sim_s'),
                   'land_s': ev.get('request_land', {}).get('sim_s'), 'disarm_s': t1,
                   'gates': [[g['gate'], g['source_sim_s']] for g in (m.get('gate_events') or [])],
                   'reference': traj, 'settings_sha256': m.get('settings_sha256'),
                   'vio_fault': (m.get('vio_feature_quality') or {}).get('fault'),
                   'track': np.round(track, 4).tolist(), 'feature_min_per_half_s': feats, **res}
            out.write(json.dumps(rec)+'\n')
            print(run.name, m.get('state'), len(track), flush=True)
        except Exception as exc:
            print(run.name, 'skipped', type(exc).__name__, exc, flush=True)
    out.close()


if __name__ == '__main__':
    main()
