"""Smooth corridor path through the PUBLISHED course map (offline; no flight).

Replaces the A* + Bezier-corner geometry (many >20 deg kinks where the drone
must stop) with one continuous, low-curvature curve at a fixed altitude.

Algorithm (elastic band with a signed-distance corridor, auditable and simple):
 1. Map.  At the flight altitude z the published obstacles (teammate
    ``Course.collides`` boxes/cylinders, the net boundary, and the tube shell
    rasterised from its published ellipse over the drone's vertical slab) are
    rasterised on a 1 cm grid.  A signed Euclidean distance transform gives
    the horizontal clearance D(x, y) to the nearest obstacle.  Nothing from the
    simulator, a recorded flight or sensors is used.
 2. Gate posts.  Each ordered route gate becomes two thin virtual obstacles: the
    gate plane beyond its published aperture (+-half_width), extended
    ``post_length`` metres.  Keeping the path ``gate_clearance`` (> drone
    radius) from these is exactly the full-envelope aperture condition used by
    ``dronebench.gate_audit`` for a level drone.  Field G(x, y).
 3. Topology / initial guess.  The existing cleared reference (A* through the
    published map) fixes the homotopy class and gate order.  It is resampled to
    N equally spaced points; launch and pad end points stay fixed.
 4. Optimisation (L-BFGS-B, analytic gradients) of the interior points:
        E = w_bend * sum |P[i-1] - 2 P[i] + P[i+1]|^2 / h^3      (~ int kappa^2 ds)
          + w_len  * sum |P[i+1] - P[i]|^2 / h                    (length, even spacing)
          + w_obs  * sum h * max(0, c_obs  - D(P[i]))^2           (corridor)
          + w_gate * sum h * max(0, c_gate - G(P[i]))^2           (gate apertures)
    i.e. the band is pulled straight/short while the corridor penalties keep it
    inside the free region shrunk by the required clearance.  Penalty
    continuation (weights x10 per round) drives violations to ~0.
 5. Output.  A periodic-free cubic spline through the optimised points is
    sampled every 5 mm; hard clearance thresholds, the kink count and the
    unchanged ``dronebench.gate_audit.audit`` are checked before writing.
    Timing is a constant-speed placeholder; retime with prepare_fast_reference.
"""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.optimize import minimize

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'tools'))
from dronebench.gate_audit import audit  # noqa: E402
from prepare_fast_reference import find_kinks  # noqa: E402


class Field:
    """Signed horizontal distance (m) to occupied cells on a square grid."""

    def __init__(self, occupied, origin, resolution):
        self.origin, self.res = float(origin), float(resolution)
        occupied = np.asarray(occupied, bool)
        self.d = (distance_transform_edt(~occupied) - distance_transform_edt(occupied))*self.res
        gy, gx = np.gradient(self.d, self.res)
        self.gx, self.gy = gx, gy

    def _ij(self, p):
        p = np.atleast_2d(p)
        return np.vstack([(p[:, 1]-self.origin)/self.res-.5, (p[:, 0]-self.origin)/self.res-.5])

    def __call__(self, p):
        return map_coordinates(self.d, self._ij(p), order=1, mode='nearest')

    def grad(self, p):
        ij = self._ij(p)
        return np.column_stack([map_coordinates(self.gx, ij, order=1, mode='nearest'),
                                map_coordinates(self.gy, ij, order=1, mode='nearest')])


def grid_centres(half_size, resolution):
    xs = np.arange(-half_size+resolution/2, half_size, resolution)
    return xs, np.meshgrid(xs, xs)


def obstacle_grid(course, *, altitude, half_height, half_size=3.048, resolution=.01):
    """Occupied cells of the horizontal slab [z-half_height, z+half_height]."""
    xs, (X, Y) = grid_centres(half_size, resolution)
    pts = np.column_stack([X.ravel(), Y.ravel(), np.full(X.size, altitude)])
    occ = course.collides(pts, 0., half_height).reshape(X.shape)
    t = course.tube
    if t is not None:
        # Thin shell: rasterise conservatively.  Inside the slab the narrowest
        # horizontal half-width of the inner ellipse is at the slab edge farthest
        # from the tube axis height.
        ez = max(abs(altitude-half_height-t['cz']), abs(altitude+half_height-t['cz']))
        inner = t['a']*np.sqrt(max(0., 1-(ez/t['b'])**2))
        ex = np.abs(X-t['cx'])
        occ |= (Y >= t['y0']) & (Y <= t['y1']) & (ex >= inner) & (ex <= t['a']+t['wall']+resolution)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True    # published net boundary
    return xs, occ


def gate_post_grid(route, xs, *, post_length=.6, resolution=.01):
    """Gate planes outside each published aperture as thin virtual walls."""
    occ = np.zeros((len(xs), len(xs)), bool)
    for i in range(len(route.centre)):
        if route.is_hover[i]:
            continue
        c, n = route.centre[i, :2], route.normal[i, :2]
        lat = np.array([-n[1], n[0]])
        for side in (-1, 1):
            u = np.arange(route.half_w[i], route.half_w[i]+post_length, resolution/2)
            q = c + side*u[:, None]*lat
            jj = np.round((q[:, 0]-xs[0])/resolution).astype(int)
            ii = np.round((q[:, 1]-xs[0])/resolution).astype(int)
            ok = (ii >= 0) & (ii < len(xs)) & (jj >= 0) & (jj < len(xs))
            occ[ii[ok], jj[ok]] = True
    return occ


def resample(points, count):
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    keep = np.r_[True, np.diff(s) > 1e-9]
    s, points = s[keep], points[keep]
    u = np.linspace(0, s[-1], count)
    return np.column_stack([np.interp(u, s, points[:, k]) for k in range(points.shape[1])])


def band_energy(x, ends, h, fields, weights):
    """Elastic-band energy and gradient for interior points x (flattened)."""
    P = np.vstack([ends[0], x.reshape(-1, 2), ends[1]])
    g = np.zeros_like(P)
    w_bend, w_len = weights['bend'], weights['len']
    dd = P[:-2]-2*P[1:-1]+P[2:]
    E = w_bend*np.sum(dd**2)/h**3
    gd = 2*w_bend*dd/h**3
    g[:-2] += gd; g[1:-1] -= 2*gd; g[2:] += gd
    d1 = np.diff(P, axis=0)
    E += w_len*np.sum(d1**2)/h
    g1 = 2*w_len*d1/h
    g[:-1] -= g1; g[1:] += g1
    for field, target, w in fields:
        D = field(P)
        short = np.maximum(0., target-D)
        E += w*h*np.sum(short**2)
        g -= (2*w*h*short)[:, None]*field.grad(P)
    return E, g[1:-1].ravel()


def optimise(initial, fields, *, spacing, w_bend, w_len, w_obs, rounds=4):
    P = resample(initial, max(3, int(round(np.sum(np.linalg.norm(np.diff(initial, axis=0), axis=1))/spacing))+1))
    ends = (P[0].copy(), P[-1].copy())
    x = P[1:-1].ravel()
    for r in range(rounds):
        h = np.sum(np.linalg.norm(np.diff(np.vstack([ends[0], x.reshape(-1, 2), ends[1]]), axis=0), axis=1))/(len(P)-1)
        w = w_obs*10**r
        fl = [(f, c, w) for f, c in fields]
        res = minimize(band_energy, x, jac=True, method='L-BFGS-B',
                       args=(ends, h, fl, {'bend': w_bend, 'len': w_len}),
                       options={'maxiter': 20000, 'maxcor': 30, 'ftol': 1e-14, 'gtol': 1e-9})
        x = res.x
        # Re-space evenly along the curve between rounds (keeps h meaningful).
        Q = resample(np.vstack([ends[0], x.reshape(-1, 2), ends[1]]), len(P))
        x = Q[1:-1].ravel()
    return np.vstack([ends[0], x.reshape(-1, 2), ends[1]])


def densify(points, step=.005):
    """C2 cubic spline (chord-length parameter) sampled every ``step`` metres."""
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    spline = CubicSpline(s, points, bc_type='natural')
    u = np.linspace(0, s[-1], int(np.ceil(s[-1]/(step/2)))+1)
    fine = spline(u)
    return resample(fine, int(np.ceil(path_len(fine)/step))+1)


def path_len(p):
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def curvature(p):
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    d1 = np.gradient(p, s, axis=0)
    d2 = np.gradient(d1, s, axis=0)
    return np.abs(d1[:, 0]*d2[:, 1]-d1[:, 1]*d2[:, 0])/np.maximum(np.linalg.norm(d1, axis=1)**3, 1e-12)


def to_rows(xy, altitude, speed=.5):
    """11-column reference: constant-speed placeholder timing, yaw = tangent."""
    p = np.column_stack([xy, np.full(len(xy), altitude)])
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    tan = np.gradient(p, s, axis=0)
    tan /= np.maximum(np.linalg.norm(tan, axis=1, keepdims=True), 1e-12)
    yaw = np.unwrap(np.arctan2(tan[:, 1], tan[:, 0]))
    v = tan*speed
    v[0] = v[-1] = 0.
    return np.column_stack([s/speed, p, v, np.cos(yaw/2), np.zeros((len(p), 2)), np.sin(yaw/2)])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--repo', type=Path, default=ROOT/'teammate-review/dronehacks2026')
    ap.add_argument('--source', type=Path, default=ROOT/'artifacts/published-course-observable-v1/robust-corners/trajectory.csv',
                    help='cleared reference whose topology (gate order, sides) seeds the band')
    ap.add_argument('--contract', type=Path, default=ROOT/'artifacts/sky-beta-estimated-v4/airframe-contract.json')
    ap.add_argument('--output', type=Path, default=ROOT/'artifacts/published-course-corridor-v1')
    ap.add_argument('--altitude', type=float, default=.9)
    ap.add_argument('--tracking-margin', type=float, default=.08, help='added to radius and half-height for the map')
    ap.add_argument('--obstacle-margin', type=float, default=.05, help='extra soft target beyond radius+tracking margin')
    ap.add_argument('--gate-margin', type=float, default=.03, help='soft target beyond radius at gate posts')
    ap.add_argument('--spacing', type=float, default=.05)
    ap.add_argument('--w-bend', type=float, default=1.)
    ap.add_argument('--w-len', type=float, default=.3)
    ap.add_argument('--w-obs', type=float, default=1e3)
    a = ap.parse_args()

    sys.path.insert(0, str(a.repo.resolve()))
    from drl.config import load_config
    config, course, route = load_config(str(a.repo.resolve()/'config/course_v3.yaml'))
    contract = json.loads(a.contract.read_text())
    radius, half_h = contract['collision_radius_m'], contract['collision_half_height_m']
    hard_obs, hard_gate = radius+a.tracking_margin, radius
    xs, occ = obstacle_grid(course, altitude=a.altitude, half_height=half_h+a.tracking_margin, half_size=course.half_size)
    res = xs[1]-xs[0]
    obs = Field(occ, xs[0]-res/2, res)
    gates = Field(gate_post_grid(route, xs) | occ, xs[0]-res/2, res)
    fields = [(obs, hard_obs+a.obstacle_margin), (gates, hard_gate+a.gate_margin)]

    src = np.loadtxt(a.source, delimiter=',', skiprows=1)
    band = optimise(src[:, 1:3], fields, spacing=a.spacing, w_bend=a.w_bend, w_len=a.w_len, w_obs=a.w_obs)
    xy = densify(band)
    rows = to_rows(xy, a.altitude)

    clear_obs, clear_gate = obs(xy), gates(xy)
    kappa = curvature(xy)
    kinks, _ = find_kinks(rows[:, 1:4])
    gate_list = [dict(name=route.names[i], center=route.centre[i].tolist(), normal=route.normal[i].tolist(),
                      half_width=float(route.half_w[i]), half_height=float(route.half_h[i]))
                 for i in range(len(route)) if not route.is_hover[i]]
    # gate_audit needs dense chronological samples; placeholder timing at 0.5 m/s, 5 mm.
    g = audit(rows[:, [0, 1, 2, 3, 7, 8, 9, 10]], gate_list, radius=radius, half_height=half_h)
    checks = {'obstacle_clearance_ok': bool(clear_obs.min() >= hard_obs),
              'gate_post_clearance_ok': bool(clear_gate.min() >= hard_gate),
              'no_kinks': not kinks, 'ordered_planes_cleared': bool(g['ordered_planes_cleared'])}
    obj = a.repo/config['course_obj']
    plan = {'kind': 'published_map_corridor_elastic_band_path_not_flown',
            'method': 'signed-distance corridor elastic band (L-BFGS-B, penalty continuation) seeded by the cleared '
                      'A* reference; natural cubic spline output at 5 mm',
            'map_source': 'published OBJ prior via teammate Course primitives; no simulator state or sensor map',
            'source_obj_sha256': sha256(obj.read_bytes()).hexdigest(),
            'seed_trajectory': str(a.source), 'seed_sha256': sha256(a.source.read_bytes()).hexdigest(),
            'generator_sha256': sha256(Path(__file__).read_bytes()).hexdigest(),
            'parameters': {'altitude_m': a.altitude, 'radius_m': radius, 'half_height_m': half_h,
                           'tracking_margin_m': a.tracking_margin, 'obstacle_target_m': hard_obs+a.obstacle_margin,
                           'gate_post_target_m': hard_gate+a.gate_margin, 'map_resolution_m': float(res),
                           'band_spacing_m': a.spacing, 'w_bend': a.w_bend, 'w_len': a.w_len, 'w_obs': a.w_obs},
            'length_m': path_len(xy), 'samples': len(xy),
            'min_obstacle_clearance_m': float(clear_obs.min()),
            'min_obstacle_clearance_at': xy[int(np.argmin(clear_obs))].tolist(),
            'min_gate_post_clearance_m': float(clear_gate.min()),
            'max_curvature_per_m': float(kappa.max()), 'min_turn_radius_m': float(1/kappa.max()),
            'kinks_over_20deg_within_2cm': len(kinks),
            'gate_events': g['events'], 'checks': checks, 'passed': all(checks.values()),
            'timing': 'placeholder constant 0.5 m/s; retime with tools/prepare_fast_reference.py',
            'requires_independent_mesh_screen': True, 'flight_completed': False}
    a.output.mkdir(parents=True, exist_ok=True)
    np.savetxt(a.output/'path.csv', rows, delimiter=',', header='t,x,y,z,vx,vy,vz,qw,qx,qy,qz', comments='')
    (a.output/'plan.json').write_text(json.dumps(plan, indent=2))
    print(json.dumps({k: v for k, v in plan.items() if k != 'gate_events'}, indent=2))
    raise SystemExit(0 if plan['passed'] else 1)


if __name__ == '__main__':
    main()
