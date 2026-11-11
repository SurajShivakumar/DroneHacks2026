"""Failure map: where along the course do flights die, at what speed, yaw rate and
heading, and what did the survivors do there? Then derive per-arc-length speed
caps ("RL on what we have": go as fast as anyone has survived at each point,
back off below where anyone died).

Input: tools/extract_run_tracks.py JSONL (truth tracks + outcomes; evaluation
data only), the route path CSV (course frame) whose arc length indexes the map.
Each truth sample after navigation start is projected onto the route (nearest
point, forward-biased local search) giving s [m]; speed, heading and yaw rate
come from the truth track. Bins of --bin-m metres collect, per bin:
  traversals        runs that passed the bin and kept flying >= --clear-m beyond
  failures          runs whose abort happened in the bin (by reason)
  survivor speeds   speeds of traversing runs in the bin; "reliable" survivors
                    (--reliable regex: references with repeated completions)
                    are tracked separately
  failure speeds    median speed over the last 1.5 s before the abort
  failure yaw rates max |yaw rate| over the last 2 s before the abort
Speed cap per bin:
  speed-related failures (speed >= --min-fail-speed) within +---fail-spread-m
                    -> min(failure speed) * --fail-factor, but never below the
                       reliable survivors' max there (proven repeatedly)
  slow failures     speed < --min-fail-speed: not a speed problem (view/yaw);
                    reported as fail_slow, no cap
  no failures       -> max survivor speed (proven at least once)
Outputs: failure-map.md, failure-map.csv, runs.csv, speed-caps.csv (s_m,v_cap)
for prepare_fast_reference.py --speed-cap-table.
"""
import argparse
import csv
import json
import re
from pathlib import Path

import numpy as np


def classify(rec):
    reason = (rec.get('reason') or '').lower()
    if rec.get('state') == 'COMPLETE':
        return 'complete'
    if 'feature' in reason or rec.get('vio_fault'):
        return 'vio'
    if 'deviation' in reason or 'tracking' in reason:
        return 'tracking'
    if 'estimator' in reason:
        return 'estimator_reset'
    return 'other'


def route_arclength(path_csv):
    d = np.loadtxt(path_csv, delimiter=',', skiprows=1)
    pts = d[:, 1:3]
    keep = np.r_[True, np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-9]
    pts = pts[keep]
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    return pts, s


def project(track_xy, pts, s, window_m=1.5):
    """Nearest route point with a forward-biased local search (no backward jumps > window)."""
    out = np.empty(len(track_xy))
    j = 0
    for i, p in enumerate(track_xy):
        lo = max(0, j-int(window_m/.005))
        hi = min(len(pts), j+int(3*window_m/.005))
        d = np.linalg.norm(pts[lo:hi]-p, axis=1)
        k = lo+int(np.argmin(d))
        # Early in the track (takeoff/hover at launch) allow a global snap.
        if i == 0:
            k = int(np.argmin(np.linalg.norm(pts-p, axis=1)))
        j = k
        out[i] = s[k]
    return out


def bin_caps(bins, *, bin_m, fail_factor, fail_spread_m, min_fail_speed, min_cap):
    """Per-bin speed cap from survivor/failure speeds (see module doc)."""
    nb = len(bins)
    caps = np.full(nb, np.nan)
    reliable = np.full(nb, np.nan)
    for i, b in enumerate(bins):
        if b['surv_speeds']:
            caps[i] = max(b['surv_speeds'])
        if b['reliable_speeds']:
            reliable[i] = max(b['reliable_speeds'])
    spread = int(round(fail_spread_m/bin_m))
    for i, b in enumerate(bins):
        fast = [v for v in b['fail_speeds'] if v >= min_fail_speed]
        if not fast:
            continue
        cap = min(fast)*fail_factor
        for j in range(max(0, i-spread), min(nb, i+spread+1)):
            c = cap if np.isnan(reliable[j]) else max(cap, reliable[j])
            caps[j] = c if np.isnan(caps[j]) else min(caps[j], c)
    last = min_cap
    for i in range(nb):
        if np.isnan(caps[i]):
            caps[i] = last
        caps[i] = max(min_cap, caps[i])
        last = caps[i]
    return caps


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--tracks', type=Path, required=True)
    p.add_argument('--route', type=Path, required=True, help='course-frame path CSV defining arc length')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--bin-m', type=float, default=.25)
    p.add_argument('--exclude', default='first-completion-v(29|3[0-7])$|course-v(8[9]|9[0-9])$',
                   help='runs in another scene appearance or that never flew (regex on run name)')
    p.add_argument('--reliable', default='course-v(44|50|51|53|81|82)$',
                   help='completed runs of references that completed repeatedly (k08, k12)')
    p.add_argument('--fail-factor', type=float, default=.8)
    p.add_argument('--fail-spread-m', type=float, default=.5)
    p.add_argument('--min-fail-speed', type=float, default=.15,
                   help='slower deaths are not speed problems (hover/view/yaw) and set no cap')
    p.add_argument('--clear-m', type=float, default=1.,
                   help='a run counts as a survivor of a bin only if it kept flying this far past it')
    p.add_argument('--min-cap', type=float, default=.12)
    a = p.parse_args()
    pts, s_route = route_arclength(a.route)
    grid = np.arange(0, s_route[-1], .005)
    pts = np.column_stack([np.interp(grid, s_route, pts[:, k]) for k in range(2)])
    s_route = grid
    tang = np.arctan2(np.gradient(pts[:, 1]), np.gradient(pts[:, 0]))
    nb = int(np.ceil(s_route[-1]/a.bin_m))
    bins = [{'s0': i*a.bin_m, 's1': (i+1)*a.bin_m, 'traversals': 0, 'surv_speeds': [], 'reliable_speeds': [],
             'surv_yaw_rates': [], 'reliable_yaw_rates': [], 'fail': {}, 'fail_speeds': [], 'fail_yaw_rates': [],
             'fail_headings': [], 'fail_runs': []} for i in range(nb)]
    runs = []
    for line in open(a.tracks):
        rec = json.loads(line)
        if re.search(a.exclude, rec['run']) or not rec.get('nav_s') or not rec.get('reference'):
            continue
        tr = np.array(rec['track'])
        nav = tr[tr[:, 0] >= rec['nav_s']]
        kind = classify(rec)
        t_end = rec.get('abort_s') or rec.get('land_s') or rec['disarm_s']
        nav = nav[nav[:, 0] <= t_end+.05]
        if len(nav) < 5:
            continue
        reliable = bool(re.search(a.reliable, rec['run'])) and kind == 'complete'
        dt = np.diff(nav[:, 0])
        sp = np.r_[np.linalg.norm(np.diff(nav[:, 1:3], axis=0), axis=1)/dt, 0]
        yr = np.r_[np.abs(np.diff(nav[:, 4]))/dt, 0]
        s = project(nav[:, 1:3], pts, s_route)
        s_fail = None
        fail_speed = fail_yr = fail_heading = None
        if kind != 'complete':
            s_fail = float(s[-1])
            k_fail = min(nb-1, int(s_fail/a.bin_m))
            b = bins[k_fail]
            b['fail'][kind] = b['fail'].get(kind, 0)+1
            fail_speed = float(np.median(sp[-16:-1]) if len(sp) > 16 else sp[:-1].mean())
            fail_yr = float(yr[max(0, len(yr)-21):-1].max())
            k = int(np.argmin(np.abs(s_route-s_fail)))
            fail_heading = float(np.degrees(np.arctan2(np.sin(nav[-1, 4]-tang[k]), np.cos(nav[-1, 4]-tang[k]))))
            b['fail_speeds'].append(fail_speed)
            b['fail_yaw_rates'].append(fail_yr)
            b['fail_headings'].append(fail_heading)
            b['fail_runs'].append(rec['run'])
        # survivor of bin i: completed, or died >= clear_m beyond the bin's end
        last_ok = nb if s_fail is None else int(np.floor((s_fail-a.clear_m)/a.bin_m))
        for i in range(nb):
            inb = (s >= bins[i]['s0']) & (s < bins[i]['s1'])
            if not inb.any() or i >= last_ok:
                continue
            bins[i]['traversals'] += 1
            bins[i]['surv_speeds'].extend(sp[inb].tolist())
            bins[i]['surv_yaw_rates'].extend(yr[inb].tolist())
            if reliable:
                bins[i]['reliable_speeds'].extend(sp[inb].tolist())
                bins[i]['reliable_yaw_rates'].extend(yr[inb].tolist())
        runs.append({'run': rec['run'], 'outcome': kind, 'reason': rec.get('reason'), 'reliable': reliable,
                     'estimator': rec.get('estimator', 'openvins'), 's_fail_m': s_fail, 's_max_m': float(s.max()),
                     'fail_speed_mps': fail_speed, 'fail_yaw_rate_rps': fail_yr, 'fail_heading_vs_tangent_deg': fail_heading,
                     'nav_s': rec['nav_s'], 'end_s': t_end, 't_fail_after_nav_s': None if s_fail is None else round(t_end-rec['nav_s'], 1),
                     'gates': len(rec.get('gates') or [])})
    a.output.mkdir(parents=True, exist_ok=True)
    with (a.output/'runs.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(runs[0].keys())); w.writeheader(); w.writerows(runs)
    caps = bin_caps(bins, bin_m=a.bin_m, fail_factor=a.fail_factor, fail_spread_m=a.fail_spread_m,
                    min_fail_speed=a.min_fail_speed, min_cap=a.min_cap)
    rows = []
    for i, b in enumerate(bins):
        ss = np.array(b['surv_speeds']) if b['surv_speeds'] else np.array([np.nan])
        rs = np.array(b['reliable_speeds']) if b['reliable_speeds'] else np.array([np.nan])
        ry = np.array(b['reliable_yaw_rates']) if b['reliable_yaw_rates'] else np.array([np.nan])
        slow = sum(v < a.min_fail_speed for v in b['fail_speeds'])
        rows.append({'s0_m': round(b['s0'], 2), 's1_m': round(b['s1'], 2), 'traversals': b['traversals'],
                     'fail_vio': b['fail'].get('vio', 0), 'fail_tracking': b['fail'].get('tracking', 0),
                     'fail_other': sum(v for k, v in b['fail'].items() if k not in ('vio', 'tracking')),
                     'fail_slow': slow,
                     'surv_speed_med': round(float(np.nanmedian(ss)), 3), 'surv_speed_max': round(float(np.nanmax(ss)), 3),
                     'reliable_speed_max': round(float(np.nanmax(rs)), 3), 'reliable_yaw_rate_max': round(float(np.nanmax(ry)), 3),
                     'fail_speeds': ';'.join(f'{v:.2f}' for v in b['fail_speeds']),
                     'fail_yaw_rates': ';'.join(f'{v:.2f}' for v in b['fail_yaw_rates']),
                     'fail_headings_deg': ';'.join(f'{v:.0f}' for v in b['fail_headings']),
                     'fail_runs': ';'.join(b['fail_runs']), 'v_cap': round(float(caps[i]), 3)})
    with (a.output/'failure-map.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    with (a.output/'speed-caps.csv').open('w', newline='') as f:
        f.write('s_m,v_cap\n')
        for i in range(nb):
            f.write(f'{bins[i]["s0"]+a.bin_m/2:.3f},{caps[i]:.3f}\n')
    n_runs = len(runs); n_complete = sum(r['outcome'] == 'complete' for r in runs)
    md = [f'# Failure map ({n_runs} flights after navigation start, {n_complete} complete)', '',
          f'Route arc length from `{a.route}`; bins {a.bin_m} m. Speeds are truth ground speeds (m/s), '
          f'yaw rates rad/s, headings relative to the route tangent (deg). Reliable = `{a.reliable}`. '
          f'Caps: failures at >= {a.min_fail_speed} m/s cap +-{a.fail_spread_m} m at {a.fail_factor} x the slowest such '
          f'failure (never below the reliable max); slower deaths are listed (fail_slow) but set no cap; '
          f'elsewhere the cap is the fastest survivor.', '',
          '| s [m] | surv | VIO | track | other | slow | surv med / max | reliable max v / yaw | fail v | fail yaw | fail hdg | cap | failed runs |',
          '|---:|---:|---:|---:|---:|---:|---|---|---|---|---|---:|---|']
    for r in rows:
        nfail = r['fail_vio']+r['fail_tracking']+r['fail_other']
        flag = ' **' if nfail else ''
        md.append(f"| {r['s0_m']:.2f}-{r['s1_m']:.2f}{flag} | {r['traversals']} | {r['fail_vio']} | {r['fail_tracking']} | {r['fail_other']} | "
                  f"{r['fail_slow']} | {r['surv_speed_med']} / {r['surv_speed_max']} | {r['reliable_speed_max']} / {r['reliable_yaw_rate_max']} | "
                  f"{r['fail_speeds']} | {r['fail_yaw_rates']} | {r['fail_headings_deg']} | {r['v_cap']} | "
                  f"{r['fail_runs'].replace('gb10-sky-beta-', '')} |")
    (a.output/'failure-map.md').write_text('\n'.join(md)+'\n', encoding='utf-8')
    hot = sorted(rows, key=lambda r: -(r['fail_vio']+r['fail_tracking']+r['fail_other']))[:10]
    print(f'{n_runs} runs, {n_complete} complete; hottest bins:')
    for r in hot:
        if r['fail_vio']+r['fail_tracking']+r['fail_other']:
            print(f"  s {r['s0_m']:.2f}-{r['s1_m']:.2f}: vio {r['fail_vio']} tracking {r['fail_tracking']} other {r['fail_other']} slow {r['fail_slow']}"
                  f" | surv {r['traversals']} med/max {r['surv_speed_med']}/{r['surv_speed_max']} reliable {r['reliable_speed_max']}"
                  f" | fail v {r['fail_speeds']} yaw {r['fail_yaw_rates']} -> cap {r['v_cap']}")


if __name__ == '__main__':
    main()
