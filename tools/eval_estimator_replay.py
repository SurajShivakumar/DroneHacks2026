"""Score an estimator's poses against simulator truth (offline evaluation only).

Inputs: estimator JSONL with t,p (and optional q, gate_ok, gate_err, obs) — e.g.
tools/cuvslam_vo_node.py --log — or OpenVINS vio-odometry.jsonl; truth-poses.csv;
course-mission.json (navigation window). Reports XY error after (a) SE3 Umeyama
alignment over the window and (b) first-sample alignment (what a live system
gets), drift per metre, and the corner1-approach slice.
"""
import argparse
import json
from pathlib import Path

import numpy as np

CORNER1 = (-2.7, -2.3, 1.2, 2.7)   # x0,x1,y0,y1 of the corner1 approach


def load_est(path):
    T, P, extra = [], [], []
    for line in open(path):
        r = json.loads(line)
        if 'position' in r:   # OpenVINS vio-odometry.jsonl
            T.append(r['source_s']); P.append([r['position'][k] for k in 'xyz']); extra.append({})
        elif r.get('valid') and 'p' in r:
            T.append(r['t']); P.append(r['p']); extra.append(r)
    return np.array(T), np.array(P), extra


def umeyama(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    H = (src-ms).T@(dst-md)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T@U.T))])
    R = Vt.T@D@U.T
    return R, md-R@ms


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--est', type=Path, required=True)
    p.add_argument('--truth', type=Path, required=True)
    p.add_argument('--mission', type=Path, required=True)
    p.add_argument('--label', default='estimator')
    a = p.parse_args()
    m = json.loads(a.mission.read_text())
    ev = {e['kind']: e for e in m['events']}
    t0 = ev['navigation_started']['sim_s']
    t1 = ev.get('request_land', {}).get('sim_s') or ev['landed_disarmed']['sim_s']
    tr = np.loadtxt(a.truth, delimiter=',', skiprows=1)
    te, pe, extra = load_est(a.est)
    k = (te >= t0) & (te <= t1)
    te, pe = te[k], pe[k]
    extra = [x for x, kk in zip(extra, k) if kk]
    pt = np.column_stack([np.interp(te, tr[:, 0], tr[:, i]) for i in (1, 2, 3)])
    R, t = umeyama(pe, pt)
    err_se3 = np.linalg.norm((pe@R.T+t-pt)[:, :2], axis=1)
    # First-sample alignment: yaw + translation from the first 1 s (rigid in xy).
    w = te <= te[0]+1.
    Rf, tf = umeyama(pe[w] if w.sum() >= 3 else pe[:3], pt[w] if w.sum() >= 3 else pt[:3])
    # Use only yaw from that fit (tilt is gravity-defined for both).
    yaw = np.arctan2(Rf[1, 0], Rf[0, 0])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    off = pt[0]-Rz@pe[0]
    err_ff = np.linalg.norm((pe@Rz.T+off-pt)[:, :2], axis=1)
    path = float(np.sum(np.linalg.norm(np.diff(pt[:, :2], axis=0), axis=1)))
    c1 = (pt[:, 0] >= CORNER1[0]) & (pt[:, 0] <= CORNER1[1]) & (pt[:, 1] >= CORNER1[2]) & (pt[:, 1] <= CORNER1[3])
    # Corner1 relative drift: error growth across the slice after aligning at its entry.
    c1_growth = None
    if c1.sum() > 3:
        i0, i1 = np.where(c1)[0][[0, -1]]
        d_est, d_tru = pe[i1]-pe[i0], pt[i1]-pt[i0]
        c1_growth = float(abs(np.linalg.norm(d_est[:2])-np.linalg.norm(d_tru[:2])))
    out = {'label': a.label, 'samples': int(len(te)), 'window_s': [t0, t1], 'truth_path_xy_m': path,
           'se3': {'xy_rms_m': float(np.sqrt(np.mean(err_se3**2))), 'xy_max_m': float(err_se3.max())},
           'first_frame': {'xy_rms_m': float(np.sqrt(np.mean(err_ff**2))), 'xy_max_m': float(err_ff.max()),
                           'xy_final_m': float(err_ff[-1]), 'drift_pct': 100*float(err_ff[-1])/max(path, 1e-6)},
           'corner1': {'samples': int(c1.sum()), 'xy_rms_ff_m': float(np.sqrt(np.mean(err_ff[c1]**2))) if c1.any() else None,
                       'path_length_error_m': c1_growth}}
    if extra and 'gate_ok' in extra[0]:
        ge = np.array([x.get('gate_err', 0.) for x in extra])
        bad = [x['t'] for x in extra if not x.get('gate_ok', True)]
        out['gate'] = {'err_p50_p99_max': [float(np.percentile(ge, 50)), float(np.percentile(ge, 99)), float(ge.max())],
                       'would_trip_at_s': (bad[0]-t0) if bad else None}
    if extra and 'obs' in extra[0]:
        ob = np.array([x['obs'] for x in extra])
        out['observations_min_p5_median'] = [int(ob.min()), float(np.percentile(ob, 5)), float(np.median(ob))]
    print(json.dumps(out, indent=1))


if __name__ == '__main__':
    main()
