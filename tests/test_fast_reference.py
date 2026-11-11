"""Continuous-motion reference generator regressions (no simulator)."""
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import prepare_fast_reference as f  # noqa: E402


def test_arc_resample_stays_on_polyline_and_drops_rests():
    pts = np.array([[0, 0, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1], [1, 1, 1]], float)
    s, p, _, _ = f.arc_resample(pts, .1)
    assert abs(s[-1]-2) < 1e-12 and np.all(np.diff(s) > 0)
    on = (np.abs(p[:, 1]) < 1e-12) | (np.abs(p[:, 0]-1) < 1e-12)
    assert on.all()


def test_speed_profile_respects_all_limits():
    s = np.linspace(0, 5, 1001)
    curv = np.where((s > 2) & (s < 2.5), 4., 0.)
    dpsi = np.where((s > 3) & (s < 3.5), 2., 0.)
    v = f.speed_profile(s, curv, dpsi, v_max=.45, a_max=.25, a_lat=.3, omega_max=.25)
    assert v[0] == 0 and v[-1] == 0 and v.max() <= .45+1e-12
    assert np.all(v[curv > 0]**2*curv[curv > 0] <= .3+1e-9)
    assert np.all(v[dpsi > 0]*dpsi[dpsi > 0] <= .25+1e-9)
    accel = np.diff(v**2)/(2*np.diff(s))
    assert np.abs(accel).max() <= .25+1e-9
