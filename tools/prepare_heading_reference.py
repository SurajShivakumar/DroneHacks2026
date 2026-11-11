"""Offline: replace a reviewed reference's fixed yaw with perception-aware headings.

Positions/velocities of every source sample are copied unchanged; the only
change is the yaw column plus stationary in-place turn dwells inserted at
existing rest samples. The Sky Beta collision envelope is a vertical cylinder,
so heading does not change mesh or gate clearance (re-run both checks anyway).

Heading choice uses only the published course geometry (teammate course_v3
boxes + OBJ net bounds) through ``dronebench.published_raycast``. It scores how
much of the forward stereo field of view would see net/exterior background at
a usable depth, penalising near-net views (one-cell stereo aliasing observed in
v29/v31/v33) and flat untextured walls. No simulator state, recorded flight or
ground truth is read. This is a planning heuristic, not a VIO qualification.
"""
import argparse
from hashlib import sha256
import json
from math import atan2, cos, pi, sin
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dronebench.published_raycast import build_published_raycast  # noqa: E402

# Default rectified pinhole (CameraInfo fx=381.36, 640x400) and zero pitch.
WIDTH, HEIGHT, FX = 640, 400, 381.36
# Score = fraction of view on net/exterior background at >= GOOD_MIN_M depth.
# Fitted to 167 flight samples (v33/v35/v37 pose vs OpenVINS SLAM count,
# Spearman 0.75): every sample scoring >=0.3 kept >=72 features; <0.15 had a
# median of 14. Flat walls and nearer net contribute nothing. Recorded views
# never exceeded ~2.1 m, so deeper thresholds are untested.
GOOD_MIN_M, GOOD_MAX_M = 2.0, 9.0
SAFE_SCORE = 0.3
YAW_RATE = 0.15         # rad/s peak dwell rate, below the 0.2 rad/s command limit
DWELL_DT = 0.01
APPROACH_SLOW_M, APPROACH_SLOW_FACTOR = 0.4, 3.0
TURN_PENALTY = 0.12     # score units per radian of in-place turn
KEEP_FLOOR, KEEP_MARGIN = SAFE_SCORE, 0.1
TURN_PENALTY = 0.03


def camera_rays(nx=24, ny=12):
    u = np.linspace(8, WIDTH-8, nx)-WIDTH/2
    v = np.linspace(8, HEIGHT-8, ny)-HEIGHT/2
    uu, vv = np.meshgrid(u, v)
    # Body FLU-like: forward x, left y, up z; optical z=1 => t is optical depth.
    return np.column_stack((np.ones(uu.size), -uu.ravel()/FX, -vv.ravel()/FX))


class ViewScorer:
    def __init__(self, geometry, textured_walls=False, surface_min_m=.5):
        self.geometry = geometry
        # Nov 6: v63-v71 lost features facing a maze divider at 0.53 m (dark,
        # near-uniform in the onboard image); callers may raise this minimum.
        self.surface_min_m = surface_min_m
        self.rays = camera_rays()
        # textured_walls: estimated surface-appearance scene, where walls and
        # floor carry texture. Uncalibrated extension of the fitted rule: count
        # wall/floor at 0.5..4 m as textured, penalise net nearer than 1.2 m
        # (v33 lost most features approaching the net at ~1-1.2 m).
        self.textured_walls = textured_walls

    def score(self, position, yaw):
        c, s = cos(yaw), sin(yaw)
        rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
        hits = self.geometry.cast(np.asarray(position, float), self.rays@rot.T)
        labels = np.array([str(l) if l is not None else '' for l in hits.labels])
        t = hits.t
        net = np.char.startswith(labels.astype(str), 'explicit_net')
        good = net & (t >= GOOD_MIN_M) & (t <= GOOD_MAX_M)
        if not self.textured_walls:
            return float(good.mean())
        surface = ~net & (labels != '') & (t >= self.surface_min_m) & (t <= 4.)
        near = net & (t < 1.2)
        return float((good | surface).mean() - near.mean())


def wrap(a):
    return atan2(sin(a), cos(a))


def rest_indices(samples, tol=1e-9):
    speed = np.linalg.norm(samples[:, 4:7], axis=1)
    return np.flatnonzero(speed <= tol)


def segments_between_rests(samples):
    rests = rest_indices(samples)
    # Collapse consecutive rest samples into rest blocks; keep block's first index.
    blocks = []
    for i in rests:
        if blocks and i == blocks[-1][-1]+1:
            blocks[-1].append(i)
        else:
            blocks.append([i])
    segments = []
    for a, b in zip(blocks[:-1], blocks[1:]):
        if b[0] > a[-1]+1:
            segments.append((a[-1], b[0]))
    return blocks, segments


def choose_headings(samples, scorer, candidates, start_yaw, forced=()):
    """forced: (xmin, xmax, ymin, ymax, yaw_rad) boxes for geometry the raycast
    omits (the curved tube shell). A segment with any sample inside a box uses
    that yaw. v39: scorer-chosen -157.5 deg faced the dark tube wall at close
    range and SLAM features collapsed; the tube axis keeps the exit in view."""
    _, segments = segments_between_rests(samples)
    chosen = []
    previous = start_yaw
    for a, b in segments:
        xy = samples[a:b+1, 1:3]
        box = next((f for f in forced if np.any((xy[:, 0] >= f[0]) & (xy[:, 0] <= f[1])
                                                 & (xy[:, 1] >= f[2]) & (xy[:, 1] <= f[3]))), None)
        if box is not None:
            idx = np.unique(np.r_[np.arange(a, b+1, max(1, (b-a)//12)), b])
            scores = [scorer.score(samples[i, 1:4], box[4]) for i in idx]
            chosen.append({'start_index': int(a), 'end_index': int(b), 'yaw_rad': float(box[4]),
                           'min_view_score': min(scores), 'mean_view_score': float(np.mean(scores)),
                           'forced_box': list(map(float, box[:4]))})
            previous = box[4]
            continue
        idx = np.unique(np.r_[np.arange(a, b+1, max(1, (b-a)//12)), b])
        best = None
        keep = None
        for yaw in list(candidates)+[previous]:
            scores = [scorer.score(samples[i, 1:4], yaw) for i in idx]
            value = min(scores) + .25*float(np.mean(scores)) - TURN_PENALTY*abs(wrap(yaw-previous))
            entry = (value, yaw, min(scores), float(np.mean(scores)))
            if yaw is previous:
                keep = entry
            if best is None or value > best[0]:
                best = entry
        # Hysteresis: an in-place turn is itself a perception risk, so keep the
        # current heading when its worst view is acceptable and nearly as good.
        if keep[2] >= KEEP_FLOOR and best[2]-keep[2] < KEEP_MARGIN:
            best = keep
        chosen.append({'start_index': int(a), 'end_index': int(b), 'yaw_rad': float(best[1]),
                       'min_view_score': best[2], 'mean_view_score': best[3]})
        previous = best[1]
    return chosen


def turn_direction(scorer, position, start, delta):
    """Pick the rotation sense whose swept views stay best (both senses scored)."""
    options = []
    for sense in (1, -1):
        d = delta if np.sign(delta) == sense or delta == 0 else delta - sense*2*pi*np.sign(delta)
        if sense*d < 0:
            d += sense*2*pi
        ts = np.linspace(0, 1, 9)
        options.append((min(scorer.score(position, start+d*x) for x in ts) - .05*abs(d), d))
    return max(options)[1]


def build(samples, chosen, scorer, start_yaw, speed_scale=1.):
    """Copy all source rows; insert dwell rows at segment-start rests for turns.

    The final APPROACH_SLOW_M of path into every turn rest is slowed by
    APPROACH_SLOW_FACTOR. v38: the tracking clock's nearest-phase progress
    floor let the aircraft arrive at a stop at ~0.2 m/s and overshoot 4.5 cm;
    through-stops absorb that along the path, but an in-place dwell measures it
    against the stop point and trips the unchanged 4 cm tracking guard.
    """
    # Pre-pass: which segment-start rests carry a turn.
    turn_rests, yaw_prev = set(), start_yaw
    for c in chosen:
        if abs(wrap(c['yaw_rad']-yaw_prev)) > 1e-6:
            turn_rests.add(c['start_index'])
        yaw_prev = c['yaw_rad']
    stretch = np.ones(len(samples))
    for k in turn_rests:
        distance, j = 0., k
        while j > 0 and distance < APPROACH_SLOW_M:
            distance += float(np.linalg.norm(samples[j, 1:4]-samples[j-1, 1:4]))
            stretch[j] = APPROACH_SLOW_FACTOR
            j -= 1
    out = []
    yaw_now = start_yaw
    starts = {c['start_index']: c for c in chosen}
    turns = []
    t_new = 0.
    for i, row in enumerate(samples):
        if i > 0:
            dt = (row[0]-samples[i-1, 0])*stretch[i]/speed_scale
            pieces = max(1, int(np.ceil(dt/DWELL_DT/2)))  # keep spacing <= 20 ms
            for m in range(1, pieces):
                f = m/pieces
                mid = samples[i-1]*(1-f)+row*f
                out.append(np.r_[t_new+dt*f, mid[1:4], mid[4:7]*speed_scale/stretch[i], yaw_now])
            t_new += dt
        velocity = row[4:7]*speed_scale/stretch[i]
        if i in starts:
            target = starts[i]['yaw_rad']
            delta = turn_direction(scorer, row[1:4], yaw_now, wrap(target-yaw_now))
            if abs(delta) > 1e-6:
                duration = max(1., 1.875*abs(delta)/YAW_RATE)  # quintic peak rate 1.875*d/T
                t0 = t_new
                out.append(np.r_[t0, row[1:4], velocity, yaw_now])
                for tau in np.arange(DWELL_DT, duration+1e-12, DWELL_DT):
                    x = min(1., tau/duration)
                    s = 10*x**3-15*x**4+6*x**5
                    out.append(np.r_[t0+tau, row[1:4], 0, 0, 0, yaw_now+delta*s])
                t_new += duration + DWELL_DT
                turns.append({'source_index': i, 'at_source_s': float(row[0]),
                              'position_m': row[1:4].tolist(), 'delta_rad': float(delta),
                              'duration_s': float(duration)})
                yaw_now += delta
                out.append(np.r_[t_new, row[1:4], velocity, yaw_now])
                continue
        out.append(np.r_[t_new, row[1:4], velocity, yaw_now])
    out = np.array(out)
    if np.any(np.diff(out[:, 0]) <= 0):
        raise ValueError('non-chronological output')
    return out, turns


def write_csv(path, rows):
    yaw = rows[:, 7]
    q = np.column_stack((np.cos(yaw/2), np.zeros(len(yaw)), np.zeros(len(yaw)), np.sin(yaw/2)))
    data = np.column_stack((rows[:, :7], q))
    np.savetxt(path, data, delimiter=',', header='t,x,y,z,vx,vy,vz,qw,qx,qy,qz', comments='',
               fmt='%.18e')


def main():
    global YAW_RATE, APPROACH_SLOW_FACTOR
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--repo', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--start-yaw', type=float, default=pi/2)
    p.add_argument('--textured-walls', action='store_true')
    p.add_argument('--force-yaw', action='append', default=[],
                   help='xmin,xmax,ymin,ymax,yaw_deg: fixed heading for segments entering this box')
    p.add_argument('--yaw-rate', type=float, default=YAW_RATE, help='peak in-place yaw rate, rad/s')
    p.add_argument('--approach-slow-factor', type=float, default=APPROACH_SLOW_FACTOR)
    p.add_argument('--speed-scale', type=float, default=1.,
                   help='uniformly speed up translation (velocities x s); dwells unaffected')
    a = p.parse_args()
    YAW_RATE, APPROACH_SLOW_FACTOR = a.yaw_rate, a.approach_slow_factor
    sys.path.insert(0, str(a.repo.resolve()))
    from drl.config import load_config
    from drl.course import load_obj_groups
    config, course, _ = load_config(str(a.repo.resolve()/'config/course_v3.yaml'))
    obj = a.repo/config['course_obj']
    _, groups = load_obj_groups(str(obj))
    net = np.concatenate([pts for name, pts in groups.items() if name.startswith('Net')])
    bounds = tuple(v for axis in range(3) for v in (net[:, axis].min(), net[:, axis].max()))
    geometry = build_published_raycast(course, published_provenance='published OBJ:'+sha256(obj.read_bytes()).hexdigest(),
                                       net_bounds=bounds, floor_z=0.)
    scorer = ViewScorer(geometry, textured_walls=a.textured_walls)
    samples = np.loadtxt(a.source, delimiter=',', skiprows=1)
    q = samples[:, 7:11]
    src_yaw = np.arctan2(2*(q[:, 0]*q[:, 3]+q[:, 1]*q[:, 2]), 1-2*(q[:, 2]**2+q[:, 3]**2))
    candidates = np.deg2rad(np.arange(-180, 180, 7.5))
    forced = [tuple(map(float, f.split(',')[:4]))+(np.deg2rad(float(f.split(',')[4])),) for f in a.force_yaw]
    chosen = choose_headings(samples, scorer, candidates, a.start_yaw, forced)
    rows, turns = build(samples, chosen, scorer, a.start_yaw, a.speed_scale)
    a.output.mkdir(parents=True, exist_ok=False)
    write_csv(a.output/'trajectory.csv', rows)
    # Baseline comparison: fixed-yaw source scored at the same sample points.
    baseline = [{'start_index': c['start_index'],
                 'min_view_score_source_yaw': min(scorer.score(samples[i, 1:4], src_yaw[i])
                                                  for i in range(c['start_index'], c['end_index']+1, 25))}
                for c in chosen]
    plan = {
        'kind': 'perception_aware_heading_reference_not_flown',
        'source_trajectory': str(a.source), 'source_sha256': sha256(a.source.read_bytes()).hexdigest(),
        'generator_sha256': sha256(Path(__file__).read_bytes()).hexdigest(),
        'obj_sha256': sha256(obj.read_bytes()).hexdigest(), 'net_bounds': list(map(float, bounds)),
        'positions_unchanged': True, 'yaw_rate_peak_rad_s': YAW_RATE, 'approach_slow_factor': APPROACH_SLOW_FACTOR, 'speed_scale': a.speed_scale,
        'scoring': {'safe_score': SAFE_SCORE, 'good_net_depth_m': [GOOD_MIN_M, GOOD_MAX_M],
                    'pinhole': [WIDTH, HEIGHT, FX], 'camera_pitch_rad': 0.},
        'textured_walls_mode': a.textured_walls, 'forced_yaw_boxes': a.force_yaw, 'segments': chosen, 'source_yaw_scores': baseline, 'turns': turns,
        'source_duration_s': float(samples[-1, 0]), 'duration_s': float(rows[-1, 0]),
        'flight_completed': False, 'finish_time_s': None,
    }
    (a.output/'plan.json').write_text(json.dumps(plan, indent=2))
    print(json.dumps({'segments': len(chosen), 'turns': len(turns), 'duration_s': plan['duration_s'],
                      'worst_new': min(c['min_view_score'] for c in chosen),
                      'worst_source': min(b['min_view_score_source_yaw'] for b in baseline)}, indent=1))
    for c, b in zip(chosen, baseline):
        print(f"seg {c['start_index']:6d}-{c['end_index']:6d} at {samples[c['start_index'],1:3].round(2)} -> "
              f"{samples[c['end_index'],1:3].round(2)} yaw {np.rad2deg(c['yaw_rad']):7.1f} "
              f"min {c['min_view_score']:+.2f} (fixed-north {b['min_view_score_source_yaw']:+.2f})")


if __name__ == '__main__':
    main()
