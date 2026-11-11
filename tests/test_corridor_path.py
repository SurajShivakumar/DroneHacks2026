"""Corridor elastic-band planner regressions (no simulator, no course files)."""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import plan_corridor_path as pc  # noqa: E402


def _wall_field():
    # 4 m square map, 1 cm cells, wall block centred at the origin (0.4 m square).
    xs = np.arange(-2+.005, 2, .01)
    X, Y = np.meshgrid(xs, xs)
    occ = (np.abs(X) < .2) & (np.abs(Y) < .2)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    return xs, pc.Field(occ, -2., .01)


def test_signed_distance_and_gradient():
    _, f = _wall_field()
    assert abs(f(np.array([[.7, 0.]]))[0]-.5) < .015
    assert f(np.array([[0., 0.]]))[0] < 0
    g = f.grad(np.array([[.7, 0.]]))[0]
    assert g[0] > .9 and abs(g[1]) < .1


def test_band_energy_gradient_matches_finite_difference():
    _, f = _wall_field()
    rng = np.random.default_rng(0)
    P = np.column_stack([np.linspace(-1, 1, 12), .3+.05*rng.standard_normal(12)])
    ends, x = (P[0], P[-1]), P[1:-1].ravel()
    fields = [(f, .45, 100.)]
    w = {'bend': 1., 'len': .3}
    E, g = pc.band_energy(x, ends, .18, fields, w)
    num = np.zeros_like(x)
    for k in range(len(x)):
        d = np.zeros_like(x); d[k] = 1e-6
        num[k] = (pc.band_energy(x+d, ends, .18, fields, w)[0]-pc.band_energy(x-d, ends, .18, fields, w)[0])/2e-6
    assert np.allclose(g, num, rtol=1e-3, atol=1e-2*np.abs(num).max())


def test_optimise_clears_obstacle_and_smooths_kink():
    _, f = _wall_field()
    seed = np.array([[-1.5, .5], [0., .5], [0., 1.5], [1.5, 1.5]])   # two 90 deg kinks
    band = pc.optimise(seed, [(f, .45)], spacing=.05, w_bend=1., w_len=.3, w_obs=1e3)
    xy = pc.densify(band)
    assert np.allclose(xy[0], seed[0]) and np.allclose(xy[-1], seed[-1], atol=1e-9)
    assert f(xy).min() >= .44
    rows = pc.to_rows(xy, .9)
    kinks, _ = pc.find_kinks(rows[:, 1:4])
    assert not kinks
    assert pc.curvature(xy).max() < 5.
    assert pc.path_len(xy) < pc.path_len(seed)


def test_gate_posts_block_outside_aperture_only():
    xs = np.arange(-2+.005, 2, .01)
    route = SimpleNamespace(centre=np.array([[0., 0., 1.]]), normal=np.array([[1., 0., 0.]]),
                            half_w=np.array([.4]), is_hover=np.array([False]))
    occ = pc.gate_post_grid(route, xs, post_length=.5)
    f = pc.Field(occ, -2., .01)
    assert f(np.array([[0., 0.]]))[0] > .39          # aperture centre is open
    assert f(np.array([[0., .6]]))[0] < .02          # post beyond the aperture
    assert f(np.array([[0., 1.5]]))[0] > .5          # posts are finite


def test_rows_are_unit_quaternion_tangent_yaw():
    xy = np.column_stack([np.linspace(0, 1, 201), np.zeros(201)])
    rows = pc.to_rows(xy, .9)
    assert rows.shape == (201, 11) and np.all(np.diff(rows[:, 0]) > 0)
    assert np.allclose(np.linalg.norm(rows[:, 7:11], axis=1), 1)
    assert np.allclose(rows[:, 3], .9) and np.allclose(rows[:, 10], 0)   # yaw 0 along +x
