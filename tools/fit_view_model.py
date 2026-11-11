"""Fit a data-driven view model: predicted OpenVINS SLAM feature count from what
the camera sees (published-map raycast) and how it moves.

Replaces the hand-set ViewScorer rule ("any wall 0.5-4 m is textured"), which
rated a dark maze divider at 0.53 m as a perfect view (v60-v71 aborts).
Inputs: tools/extract_feature_dataset.py CSV (truth poses + recorded feature
counts, used offline only to fit a planning cost). Outputs a pickled sklearn
model + metadata. Validation is leave-whole-runs-out.

Descriptor per (position, yaw, velocity, yaw_rate):
  fraction of a 16x8 ray grid hitting {net, wall/box, floor} in range bins
  [0,.6) [.6,1) [1,2) [2,4) [4,9) m, fraction missing, in-tube flag,
  speed, |yaw rate|, translational flow px/frame (median), rotational flow.
"""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import pickle
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import prepare_heading_reference as phr  # noqa: E402

BINS = (0., .6, 1., 2., 4., 9.)
CLASSES = ('net', 'wall', 'floor')
TUBE_BOX = (1.6, 3.0, -.2, 1.25)
FPS = 15.
NAMES = ([f'{c}_{BINS[i]}_{BINS[i+1]}' for c in CLASSES for i in range(len(BINS)-1)]
         + ['miss', 'in_tube', 'speed', 'yaw_rate_abs', 'flow_trans_px', 'flow_rot_px'])


def rays(nx=16, ny=8):
    u = np.linspace(8, phr.WIDTH-8, nx)-phr.WIDTH/2
    v = np.linspace(8, phr.HEIGHT-8, ny)-phr.HEIGHT/2
    uu, vv = np.meshgrid(u, v)
    return np.column_stack((np.ones(uu.size), -uu.ravel()/phr.FX, -vv.ravel()/phr.FX))


def descriptor(geometry, R, position, yaw, velocity, yaw_rate):
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
    world = R@rot.T
    hits = geometry.cast(np.asarray(position, float), world)
    labels = np.array([str(l) if l is not None else '' for l in hits.labels])
    norm = np.linalg.norm(world, axis=1)
    rng = np.where(np.isfinite(hits.t), hits.t*norm, np.inf)
    cls = np.where(np.char.startswith(labels, 'explicit_net'), 0,
                   np.where(labels == 'explicit_floor', 2, np.where(labels != '', 1, -1)))
    out = []
    for k in range(3):
        for i in range(len(BINS)-1):
            out.append(float(np.mean((cls == k) & (rng >= BINS[i]) & (rng < BINS[i+1]))))
    out.append(float(np.mean((cls < 0) | (rng >= BINS[-1]))))
    x, y = position[0], position[1]
    out.append(float(TUBE_BOX[0] <= x <= TUBE_BOX[1] and TUBE_BOX[2] <= y <= TUBE_BOX[3]))
    v = np.asarray(velocity, float)
    speed = float(np.linalg.norm(v[:2]))
    out += [speed, abs(float(yaw_rate))]
    ok = np.isfinite(rng) & (rng < BINS[-1])
    if ok.any() and speed > 1e-3:
        d = world[ok]/norm[ok, None]
        u = np.r_[v[:2], 0.]/speed
        lateral = np.linalg.norm(u[None, :]-(d@u)[:, None]*d, axis=1)
        flow = phr.FX*np.median(speed*lateral/np.maximum(rng[ok], .05))/FPS
    else:
        flow = 0.
    out += [float(flow), float(phr.FX*abs(yaw_rate)/FPS)]
    return np.array(out)


def load_geometry(repo):
    sys.path.insert(0, str(Path(repo).resolve()))
    from drl.config import load_config
    from drl.course import load_obj_groups
    config, course, _ = load_config(str(Path(repo).resolve()/'config/course_v3.yaml'))
    obj = Path(repo)/config['course_obj']
    _, groups = load_obj_groups(str(obj))
    net = np.concatenate([pts for name, pts in groups.items() if name.startswith('Net')])
    bounds = tuple(v for axis in range(3) for v in (net[:, axis].min(), net[:, axis].max()))
    return phr.build_published_raycast(course, published_provenance='published OBJ:'+sha256(obj.read_bytes()).hexdigest(),
                                       net_bounds=bounds, floor_z=0.)


class LearnedViewScorer:
    """Drop-in for ViewScorer.score(position, yaw): returns a 0..1 view quality
    from the predicted feature count at a nominal planning speed along `yaw`."""
    def __init__(self, geometry, model_path, nominal_speed=.6, good_features=40.):
        self.geometry = geometry
        blob = pickle.loads(Path(model_path).read_bytes())
        self.model, self.meta = blob['model'], blob['meta']
        self.R = rays()
        self.rays = phr.camera_rays()  # for image_flow compatibility
        self.nominal_speed, self.good = nominal_speed, good_features

    def predict(self, position, yaw, velocity=(0., 0., 0.), yaw_rate=0.):
        x = descriptor(self.geometry, self.R, position, yaw, velocity, yaw_rate)
        return float(np.expm1(self.model.predict(x[None])[0]))

    def score(self, position, yaw):
        # Static view quality (stationary, no rotation): flow is handled by the
        # speed profile's image-flow cap.
        f = self.predict(position, yaw)
        return float(np.clip(f/self.good, 0., 1.))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--repo', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--exclude', default=r'first-completion-v(29|3[0-7])$',
                   help='runs flown in a different scene appearance (pre-textured-wall)')
    a = p.parse_args()
    import csv
    from sklearn.ensemble import HistGradientBoostingRegressor
    from sklearn.model_selection import GroupKFold
    import re
    rows = [r for r in csv.DictReader(open(a.dataset)) if not re.search(a.exclude, r['run'])]
    geometry = load_geometry(a.repo)
    R = rays()
    X = np.array([descriptor(geometry, R, (float(r['x']), float(r['y']), float(r['z'])), float(r['yaw']),
                             (float(r['vx']), float(r['vy']), 0.), float(r['yaw_rate'])) for r in rows])
    yv = np.array([float(r['features']) for r in rows])
    runs = np.array([r['run'] for r in rows])
    airborne = np.array([float(r['z']) for r in rows]) > .4
    X, yv, runs = X[airborne], yv[airborne], runs[airborne]
    y = np.log1p(yv)
    pred = np.zeros_like(y)
    for tr, te in GroupKFold(n_splits=min(5, len(set(runs)))).split(X, y, runs):
        m = HistGradientBoostingRegressor(max_iter=300, learning_rate=.05, max_leaf_nodes=31, l2_regularization=1.)
        m.fit(X[tr], y[tr]); pred[te] = m.predict(X[te])
    resid = y-pred
    r2 = 1-np.sum(resid**2)/np.sum((y-y.mean())**2)
    low = yv < 15
    from sklearn.metrics import roc_auc_score
    auc = float(roc_auc_score(low, -pred)) if 0 < low.sum() < len(low) else None
    model = HistGradientBoostingRegressor(max_iter=300, learning_rate=.05, max_leaf_nodes=31, l2_regularization=1.).fit(X, y)
    a.output.mkdir(parents=True, exist_ok=True)
    meta = {'kind': 'learned_view_model_planning_cost_not_estimator', 'descriptor': NAMES, 'target': 'log1p(OpenVINS SLAM finite feature count)',
            'samples': int(len(y)), 'runs': sorted(set(runs.tolist())), 'dataset_sha256': sha256(a.dataset.read_bytes()).hexdigest(),
            'cv': {'scheme': 'GroupKFold by run', 'r2_log': float(r2), 'auc_low_lt15': auc,
                   'median_abs_err_features': float(np.median(np.abs(np.expm1(pred)-yv)))},
            'generator_sha256': sha256(Path(__file__).read_bytes()).hexdigest()}
    (a.output/'model.pkl').write_bytes(pickle.dumps({'model': model, 'meta': meta}))
    (a.output/'meta.json').write_text(json.dumps(meta, indent=1))
    np.savez_compressed(a.output/'cv-predictions.npz', y=yv, pred=np.expm1(pred), runs=runs)
    print(json.dumps(meta['cv'], indent=1), 'samples', len(y), 'runs', len(set(runs)))
    # Sanity: launch views.
    for deg in (90, 45, 0, -90):
        x = descriptor(geometry, R, (-2.515, .914, .9), np.deg2rad(deg), (0, 0, 0), 0.)
        print('launch yaw', deg, 'predicted features', round(float(np.expm1(model.predict(x[None])[0])), 1))


if __name__ == '__main__':
    main()
