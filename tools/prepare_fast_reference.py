"""Continuous-motion perception-aware reference (offline; no flight commands).

Pipeline (positions are the cleared source path, unchanged in space):
 1. Resample the source path by arc length (stationary rows dropped).
 2. Heading by dynamic programming over (station, yaw): maximise the published
    -geometry view score (the same ViewScorer that chose v40's headings),
    heavily penalise views below SAFE_SCORE, penalise turning, and bound the
    heading change per metre so turns are spread over motion instead of
    in-place dwells (v41 lost features while turning stationary at the tube).
    Forced boxes (tube) fix the heading where the raycast omits geometry.
 3. Speed profile: v(s) <= v_max, lateral-accel curvature limit, yaw-rate limit
    omega_max/|dpsi/ds|, then forward/backward acceleration passes; rest at both
    ends.
 4. Sample at 10 ms with velocity feedforward and yaw quaternion.
"""
import argparse
from hashlib import sha256
import json
from math import pi
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import prepare_heading_reference as phr  # noqa: E402

def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def arc_resample(points, ds):
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    keep = np.r_[True, seg > 1e-9]
    pts = points[keep]
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    grid = np.r_[np.arange(0, s[-1], ds), s[-1]]
    return grid, np.column_stack([np.interp(grid, s, pts[:, k]) for k in range(3)]), s, pts


def plan_yaw(stations, scorer, *, start_yaw, end_yaw, max_turn_per_m, ds, forced, turn_cost, step_deg=5.,
             extra_cost=None, start_turn_cost=.05):
    cands = np.deg2rad(np.arange(-180, 180, step_deg))
    n, m = len(stations), len(cands)
    score = np.empty((n, m))
    for i, p in enumerate(stations):
        box = next((f for f in forced if f[0] <= p[0] <= f[1] and f[2] <= p[1] <= f[3]), None)
        for j, y in enumerate(cands):
            if box is not None:
                score[i, j] = 1. if abs(wrap(y-box[4])) < 1e-6 else -10.
            else:
                score[i, j] = scorer.score(p, y)
    node = (1-score) + 8.*np.maximum(0., phr.SAFE_SCORE-score)
    if extra_cost is not None:
        node = node + np.array([[extra_cost(i, y) for y in cands] for i in range(n)])
    diff = np.abs(wrap(cands[:, None]-cands[None, :]))
    allowed = diff <= max_turn_per_m*ds+1e-9
    edge = np.where(allowed, turn_cost*diff, np.inf)
    cost = node[0] + start_turn_cost*np.abs(wrap(cands-start_yaw))
    back = np.zeros((n, m), int)
    for i in range(1, n):
        total = cost[:, None] + edge          # from j -> to k
        back[i] = np.argmin(total, axis=0)
        cost = total[back[i], np.arange(m)] + node[i]
    cost = cost + 0.05*np.abs(wrap(cands-end_yaw))
    k = int(np.argmin(cost))
    path = [k]
    for i in range(n-1, 0, -1):
        k = back[i, k]
        path.append(k)
    idx = np.array(path[::-1])
    return np.unwrap(cands[idx]), score[np.arange(n), idx]


def find_kinks(pos, step=4, min_deg=20.):
    """Stations where the raw source path changes direction by > min_deg within
    +-step samples (source timing stopped there; v42/v43 overshot one at speed)."""
    n = len(pos)
    ang = np.zeros(n)
    for i in range(step, n-step):
        a, b = pos[i]-pos[i-step], pos[i+step]-pos[i]
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na > 1e-9 and nb > 1e-9:
            ang[i] = np.degrees(np.arccos(np.clip(a@b/(na*nb), -1, 1)))
    kinks, i = [], 0
    while i < n:
        if ang[i] > min_deg:
            j = i
            while j+1 < n and ang[j+1] > min_deg:
                j += 1
            kinks.append(int(i+np.argmax(ang[i:j+1])))
            i = j+1
        else:
            i += 1
    return kinks, ang


def speed_profile(s, curvature, dpsi_ds, *, v_max, a_max, a_lat, omega_max, v_floor=0., stops=(), v_cap=None, soft_stop=0.):
    v = np.full(len(s), v_max)
    if v_cap is not None:
        v = np.minimum(v, v_cap)
    v = np.minimum(v, np.sqrt(a_lat/np.maximum(curvature, 1e-9)))
    v = np.minimum(v, omega_max/np.maximum(np.abs(dpsi_ds), 1e-9))
    v = np.maximum(v, v_floor)
    v[0] = v[-1] = 0.
    v[list(stops)] = soft_stop
    ds = np.diff(s)
    for i in range(1, len(s)):
        v[i] = min(v[i], np.sqrt(v[i-1]**2+2*a_max*ds[i-1]))
    for i in range(len(s)-2, -1, -1):
        v[i] = min(v[i], np.sqrt(v[i+1]**2+2*a_max*ds[i]))
    return v


def start_turn(d0, omega, profile='smoothstep', ramp_s=1.5):
    """In-place start turn of signed angle d0 at peak yaw rate omega.

    Returns (duration, angle_at(tau)) with angle_at(0) = 0 and angle_at(duration) = d0.
    smoothstep: quintic ease (peak rate = omega at mid-turn), 1.875*|d0|/omega.
    trapezoid: linear yaw-rate ramps of ramp_s to omega, hold, ramp down:
    |d0|/omega + ramp_s (falls back to a triangle when |d0| < omega*ramp_s).
    """
    mag = abs(d0)
    sign = 1. if d0 >= 0 else -1.
    if profile == 'trapezoid':
        if mag < omega*ramp_s:
            # triangle: two ramps of r seconds each, peak rate mag/r <= omega
            r = ramp_s if ramp_s > 0 else 1.
            peak = mag/r
            dur = 2*r

            def angle_at(tau):
                tau = min(max(tau, 0.), dur)
                if tau < r:
                    return sign*.5*peak/r*tau**2
                u = dur-tau
                return sign*(mag-.5*peak/r*u**2)
            return dur, angle_at
        dur = mag/omega+ramp_s

        def angle_at(tau):
            tau = min(max(tau, 0.), dur)
            if tau < ramp_s:
                return sign*.5*omega/ramp_s*tau**2
            if tau < dur-ramp_s:
                return sign*(.5*omega*ramp_s+omega*(tau-ramp_s))
            u = dur-tau
            return sign*(mag-.5*omega/ramp_s*u**2)
        return dur, angle_at
    dur = max(1., 1.875*mag/omega)

    def angle_at(tau):
        x = min(max(tau/dur, 0.), 1.)
        return d0*(10*x**3-15*x**4+6*x**5)
    return dur, angle_at


def time_of(s, v):
    ds = np.diff(s)
    vm = .5*(v[1:]+v[:-1])
    dt = np.where(vm > 1e-9, ds/np.maximum(vm, 1e-9), 0.)
    # rest-to-motion first/last steps: constant-accel solution
    return np.r_[0, np.cumsum(np.maximum(dt, 0.))]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--repo', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--start-yaw', type=float, default=pi/2)
    p.add_argument('--v-max', type=float, default=.45)
    p.add_argument('--a-max', type=float, default=.25)
    p.add_argument('--a-lat', type=float, default=.3)
    p.add_argument('--omega-max', type=float, default=.25)
    p.add_argument('--max-turn-per-m', type=float, default=2.)
    p.add_argument('--turn-cost', type=float, default=.15)
    p.add_argument('--yaw-ds', type=float, default=.1)
    p.add_argument('--force-yaw', action='append', default=[])
    p.add_argument('--smooth-m', type=float, default=.1, help='Hann window for path smoothing (re-audit!)')
    p.add_argument('--flow-max-px', type=float, default=None,
                   help='cap translational image flow (median, px/frame at 15 Hz) via speed limit')
    p.add_argument('--flow-weight', type=float, default=0.,
                   help='heading DP penalty per unit excess flow at --v-max (needs --flow-max-px)')
    p.add_argument('--surface-min-m', type=float, default=.5, help='nearest wall distance counted as textured')
    p.add_argument('--view-model', type=Path, default=None,
                   help='learned feature-count model (tools/fit_view_model.py): heading score + speed cap')
    p.add_argument('--good-features', type=float, default=40., help='predicted count mapped to view score 1')
    p.add_argument('--min-features', type=float, default=None,
                   help='speed cap: fastest speed whose predicted feature count stays >= this')
    p.add_argument('--start-omega', type=float, default=None, help='yaw rate for the in-place start turn')
    p.add_argument('--heading-from', type=Path, default=None,
                   help='copy the yaw profile (by arc length) from an existing reference on the same path; '
                        'skips the heading DP so only timing changes')
    p.add_argument('--baseline-ref', type=Path, default=None,
                   help='proven reference on the same path: never slower than it, and only faster while the '
                        'predicted feature count stays >= min(--min-features, baseline prediction there)')
    p.add_argument('--speed-cap-table', type=Path, default=None,
                   help='CSV s_m,v_cap from tools/failure_map.py: hard per-arc-length speed cap (learned from flights)')
    p.add_argument('--slow-zone', action='append', default=[],
                   help='x,y,r: within r of (x,y) cap speed at the --baseline-ref speed (observed failure spots)')
    p.add_argument('--soft-kink-speed', type=float, default=0.,
                   help='speed at kink stations instead of a full stop (with --stop-at-kinks)')
    p.add_argument('--start-hold-s', type=float, default=0., help='stationary hold after the start turn')
    p.add_argument('--start-turn-profile', choices=['smoothstep', 'trapezoid'], default='smoothstep',
                   help='in-place start turn: smoothstep peaks at omega for an instant (1.875x the minimum '
                        'time; k12 spent 11.6 s on 88 deg); trapezoid holds omega with --start-turn-ramp-s ramps')
    p.add_argument('--start-turn-ramp-s', type=float, default=1.5, help='yaw-rate ramp time of the trapezoid start turn')
    p.add_argument('--start-turn-m', type=float, default=0.,
                   help='> 0: no in-place turn; blend the launch heading into the plan over the first this many '
                        'metres while moving (the omega_max/|dpsi/ds| cap slows translation to keep yaw rate legal)')
    p.add_argument('--start-turn-cost', type=float, default=.05,
                   help='heading DP cost per rad of in-place turn from the launch heading')
    p.add_argument('--stop-at-kinks', action='store_true',
                   help='come to rest where the raw source path turns >20 deg within 2 cm')
    a = p.parse_args()
    sys.path.insert(0, str(a.repo.resolve()))
    from drl.config import load_config
    from drl.course import load_obj_groups
    config, course, _ = load_config(str(a.repo.resolve()/'config/course_v3.yaml'))
    obj = a.repo/config['course_obj']
    _, groups = load_obj_groups(str(obj))
    net = np.concatenate([pts for name, pts in groups.items() if name.startswith('Net')])
    bounds = tuple(v for axis in range(3) for v in (net[:, axis].min(), net[:, axis].max()))
    geometry = phr.build_published_raycast(course, published_provenance='published OBJ:'+sha256(obj.read_bytes()).hexdigest(),
                                           net_bounds=bounds, floor_z=0.)
    if a.view_model:
        import fit_view_model
        scorer = fit_view_model.LearnedViewScorer(geometry, a.view_model, good_features=a.good_features)
    else:
        scorer = phr.ViewScorer(geometry, textured_walls=True, surface_min_m=a.surface_min_m)
    src = np.loadtxt(a.source, delimiter=',', skiprows=1)
    forced = [tuple(map(float, f.split(',')[:4]))+(np.deg2rad(float(f.split(',')[4])),) for f in a.force_yaw]

    # Coarse stations for the heading DP, fine grid for timing.
    s_c, p_c, s_src, p_src = arc_resample(src[:, 1:4], a.yaw_ds)
    import image_flow
    extra = None
    if a.flow_max_px and a.flow_weight > 0:
        tan_c = np.gradient(p_c, s_c, axis=0)
        def extra(i, y):
            f = image_flow.flow_px_per_frame_per_mps(geometry, scorer.rays, p_c[i], y, tan_c[i])*a.v_max
            return a.flow_weight*max(0., f-a.flow_max_px)/a.flow_max_px
    if a.heading_from:
        ref = np.loadtxt(a.heading_from, delimiter=',', skiprows=1)
        ref_yaw = np.unwrap(2*np.arctan2(ref[:, 10], ref[:, 7]))
        step = np.r_[0, np.linalg.norm(np.diff(ref[:, 1:4], axis=0), axis=1)]
        ref_s = np.cumsum(step)
        moving = np.r_[True, step[1:] > 1e-9]
        # Yaw at each arc length; at stationary turns use the yaw *after* the
        # turn (the reference finishes turning before moving on).
        s_u, last = np.unique(ref_s[::-1], return_index=True)
        yaw_at = ref_yaw[::-1][last]
        yaw_c = np.interp(s_c*(ref_s[-1]/s_c[-1]), s_u, yaw_at)
        # Forced boxes override the copied headings (failure map: fm2 re-aims the
        # first 1.4 m); keep the unwrapped series continuous around each box.
        for i, q in enumerate(p_c):
            box = next((f for f in forced if f[0] <= q[0] <= f[1] and f[2] <= q[1] <= f[3]), None)
            if box is not None:
                yaw_c[i] = yaw_c[i]+wrap(box[4]-yaw_c[i])
        score_c = np.array([scorer.score(p_c[i], yaw_c[i]) for i in range(len(p_c))])
        a.start_yaw = float(ref_yaw[0])
    else:
        yaw_c, score_c = plan_yaw(p_c, scorer, start_yaw=a.start_yaw, end_yaw=a.start_yaw,
                                  max_turn_per_m=a.max_turn_per_m, ds=a.yaw_ds, forced=forced, turn_cost=a.turn_cost,
                                  extra_cost=extra, start_turn_cost=a.start_turn_cost)
    fine = .005
    s, pos, _, _ = arc_resample(src[:, 1:4], fine)
    yaw = np.interp(s, s_c, yaw_c)
    # Smooth heading over ~0.3 m so yaw rate is continuous; keep forced boxes exact.
    w = int(.3/fine) | 1
    kernel = np.hanning(w); kernel /= kernel.sum()
    padded = np.r_[np.full(w//2, yaw[0]), yaw, np.full(w//2, yaw[-1])]
    yaw = np.convolve(padded, kernel, mode='valid')
    dpsi = np.gradient(yaw, s)
    kinks, kink_angle = find_kinks(pos) if a.stop_at_kinks else ([], None)
    # Curvature from a 10 cm-smoothed copy of the path: the source polyline has
    # mm-scale kinks that otherwise read as ~2 m/s^2 lateral demands (v42 abort).
    wc = int(a.smooth_m/fine) | 1
    kc = np.hanning(wc); kc /= kc.sum()
    smooth = np.column_stack([np.convolve(np.r_[np.full(wc//2, pos[0, k]), pos[:, k], np.full(wc//2, pos[-1, k])], kc, mode='valid')
                              for k in range(3)])
    # Fly the smoothed path itself (corner cut <= ~1 cm; clearance and gate
    # audits are re-run on the output), re-parameterised by its own arc length.
    pos = smooth
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))]
    tangent = np.gradient(smooth, s, axis=0)
    curv = np.linalg.norm(np.gradient(tangent/np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9), s, axis=0), axis=1)
    v_cap, flow_coef = None, None
    if a.flow_max_px:
        unit0 = tangent/np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        idx = np.unique(np.r_[np.arange(0, len(s), 10), len(s)-1])
        coef = np.array([image_flow.flow_px_per_frame_per_mps(geometry, scorer.rays, pos[i], yaw[i], unit0[i]) for i in idx])
        flow_coef = np.interp(s, s[idx], coef)
        v_cap = a.flow_max_px/np.maximum(flow_coef, 1e-6)
    feat_cap, pred_at_cap = None, None
    if a.view_model and a.min_features:
        unit0 = tangent/np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        idx = np.unique(np.r_[np.arange(0, len(s), 20), len(s)-1])
        speeds = [v_ for v_ in (1.2, 1.0, .8, .65, .5, .4, .3, .2, .12) if v_ <= a.v_max+1e-9]
        base_v = base_f = None
        if a.baseline_ref:
            b = np.loadtxt(a.baseline_ref, delimiter=',', skiprows=1)
            bs = np.cumsum(np.r_[0, np.linalg.norm(np.diff(b[:, 1:4], axis=0), axis=1)])
            bv = np.linalg.norm(b[:, 4:7], axis=1)
            mv = np.r_[True, np.diff(bs) > 1e-9]
            base_v = np.interp(s*(bs[-1]/s[-1]), bs[mv], bv[mv])
            base_f = {}
        caps, preds = [], []
        for i in idx:
            floor = a.min_features
            vb = 0.
            if base_v is not None:
                vb = float(base_v[i])
                fb = scorer.predict(pos[i], yaw[i], unit0[i]*vb, vb*dpsi[i]) if vb > .02 else float('inf')
                floor = min(a.min_features, fb)
            chosen = None
            for v_ in speeds:
                f = scorer.predict(pos[i], yaw[i], unit0[i]*v_, v_*dpsi[i])
                if f >= floor:
                    chosen = (v_, f); break
            if chosen is None:
                chosen = (.12, float('nan'))
            caps.append(max(chosen[0], vb)); preds.append(chosen[1])
        feat_cap = np.interp(s, s[idx], caps)
        if a.slow_zone and base_v is not None:
            for z in a.slow_zone:
                zx, zy, zr = map(float, z.split(','))
                near = np.hypot(pos[:, 0]-zx, pos[:, 1]-zy) <= zr
                feat_cap[near] = np.minimum(feat_cap[near], np.maximum(base_v[near], .05))
        v_cap = feat_cap if v_cap is None else np.minimum(v_cap, feat_cap)
        pred_at_cap = np.array(preds)
    if a.start_turn_m > 0:
        # Moving start turn (failure map: the first metre cost every completion ~16 s,
        # 12 s of it hovering through the in-place turn; fast in-place turns died).
        d0_move = wrap(yaw[0]-a.start_yaw)
        blend = np.clip(s/a.start_turn_m, 0, 1)
        yaw = yaw - d0_move*(1-(10*blend**3-15*blend**4+6*blend**5))
        dpsi = np.gradient(yaw, s)
    table_cap = None
    if a.speed_cap_table:
        tab = np.loadtxt(a.speed_cap_table, delimiter=',', skiprows=1)
        table_cap = np.interp(s, tab[:, 0], tab[:, 1])
        v_cap = table_cap if v_cap is None else np.minimum(v_cap, table_cap)
    v = speed_profile(s, curv, dpsi, v_max=a.v_max, a_max=a.a_max, a_lat=a.a_lat, omega_max=a.omega_max, v_floor=.02,
                      stops=kinks, v_cap=v_cap, soft_stop=a.soft_kink_speed)
    t = time_of(s, v)
    unit = np.gradient(pos, s, axis=0)
    unit /= np.maximum(np.linalg.norm(unit, axis=1, keepdims=True), 1e-9)
    # Initial heading change from the launch yaw to the plan's first yaw, in place.
    rows = []
    d0 = wrap(yaw[0]-a.start_yaw)
    t_off = 0.
    if abs(d0) > 1e-6:
        om = a.start_omega or a.omega_max
        dur, angle_at = start_turn(d0, om, a.start_turn_profile, a.start_turn_ramp_s)
        for tau in np.arange(0, dur, .01):
            rows.append(np.r_[tau, pos[0], 0, 0, 0, a.start_yaw+angle_at(tau)])
        t_off = dur
        yaw = yaw - yaw[0] + (a.start_yaw+d0)
    # Optional stationary hold after the start turn so VIO re-converges before
    # accelerating (v55/v66-v68: features dipped to 3-7 in the first ~12 s).
    for tau in np.arange(0, a.start_hold_s, .01):
        rows.append(np.r_[t_off+tau, pos[0], 0, 0, 0, a.start_yaw+d0])
    t_off += a.start_hold_s
    tt = np.arange(0, t[-1], .01)
    tt = np.r_[tt, t[-1]]
    P = np.column_stack([np.interp(tt, t, pos[:, k]) for k in range(3)])
    Y = np.interp(tt, t, yaw)
    speed = np.interp(tt, t, v)
    V = np.column_stack([np.interp(tt, t, unit[:, k]) for k in range(3)])*speed[:, None]
    for i in range(len(tt)):
        rows.append(np.r_[t_off+tt[i], P[i], V[i], Y[i]])
    rows = np.array(rows)
    keep = np.r_[True, np.diff(rows[:, 0]) > 1e-9]
    rows = rows[keep]
    a.output.mkdir(parents=True, exist_ok=False)
    phr.write_csv(a.output/'trajectory.csv', rows)
    yaw_rate = np.abs(np.diff(rows[:, 7]))/np.diff(rows[:, 0])
    acc = np.gradient(rows[:, 4:7], rows[:, 0], axis=0)
    spd = np.linalg.norm(rows[:, 4:7], axis=1)
    uu = rows[:, 4:7]/np.maximum(spd[:, None], 1e-9)
    along = (acc*uu).sum(1)
    lateral = np.linalg.norm(acc-along[:, None]*uu, axis=1)
    mv = spd > .02
    # Score the final sampled headings (what will actually be commanded).
    idx = np.arange(0, len(rows), 50)
    final_scores = []
    for i in idx:
        q = rows[i, 1:4]
        box = next((f for f in forced if f[0] <= q[0] <= f[1] and f[2] <= q[1] <= f[3]), None)
        final_scores.append(None if box is not None else scorer.score(q, rows[i, 7]))
    fs = np.array([x for x in final_scores if x is not None])
    plan = {'kind': 'continuous_perception_aware_reference_not_flown',
            'source_trajectory': str(a.source), 'source_sha256': sha256(a.source.read_bytes()).hexdigest(),
            'generator_sha256': sha256(Path(__file__).read_bytes()).hexdigest(),
            'scorer_generator_sha256': sha256(Path(phr.__file__).read_bytes()).hexdigest(),
            'positions': f'source path resampled by arc length and smoothed with a {a.smooth_m*100:.0f} cm Hann window (re-audited); timing replaced',
            'path_length_m': float(s[-1]), 'duration_s': float(rows[-1, 0]),
            'limits': {'v_max': a.v_max, 'a_max': a.a_max, 'a_lat': a.a_lat, 'omega_max': a.omega_max,
                       'max_turn_per_m': a.max_turn_per_m, 'turn_cost': a.turn_cost},
            'peak_speed_mps': float(np.linalg.norm(rows[:, 4:7], axis=1).max()),
            'peak_yaw_rate_rad_s': float(yaw_rate.max()),
            'accel_lateral_p99_max': [float(np.percentile(lateral[mv], 99)), float(lateral[mv].max())],
            'accel_along_p99_max': [float(np.percentile(np.abs(along[mv]), 99)), float(np.abs(along[mv]).max())],
            'total_turn_rad': float(np.abs(np.diff(rows[:, 7])).sum()),
            'view_score_min': float(fs.min()), 'view_score_p10': float(np.percentile(fs, 10)),
            'view_score_fraction_below_safe': float((fs < phr.SAFE_SCORE).mean()),
            'kink_stops': [{'s_m': float(s[k]), 'position_m': pos[k].tolist(), 'angle_deg': float(kink_angle[k])} for k in kinks],
            'flow_limit': None if flow_coef is None else {'max_px_per_frame': a.flow_max_px, 'dp_weight': a.flow_weight,
                'planned_flow_px_p50_p90_max': [float(x) for x in np.percentile(flow_coef*v, [50, 90, 100])]},
            'start_turn_rad': float(abs(d0)), 'start_turn_profile': a.start_turn_profile,
            'start_turn_moving_m': a.start_turn_m, 'start_turn_s': float(t_off-a.start_hold_s),
            'speed_cap_table': None if not a.speed_cap_table else {'path': str(a.speed_cap_table), 'sha256': sha256(Path(a.speed_cap_table).read_bytes()).hexdigest(),
                                                                  'cap_p10_p50_max': [float(np.percentile(table_cap, 10)), float(np.percentile(table_cap, 50)), float(table_cap.max())]},
            'learned_view_model': None if not a.view_model else {'path': str(a.view_model), 'sha256': sha256(Path(a.view_model).read_bytes()).hexdigest(),
                'min_features': a.min_features, 'good_features': a.good_features, 'slow_zones': a.slow_zone,
                'speed_cap_p10_p50': None if feat_cap is None else [float(np.percentile(feat_cap, 10)), float(np.percentile(feat_cap, 50))]},
            'forced_yaw_boxes': a.force_yaw, 'flight_completed': False, 'finish_time_s': None}
    (a.output/'plan.json').write_text(json.dumps(plan, indent=2))
    print(json.dumps(plan, indent=1))


if __name__ == '__main__':
    main()
