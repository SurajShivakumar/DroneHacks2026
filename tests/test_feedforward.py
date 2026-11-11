"""Supervisor feedforward helpers (no ROS/simulator)."""
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
pytest.importorskip('px4_msgs', reason='course_flight imports ROS message types')
from tools.course_flight import bounded_feedforward, reference_rates  # noqa: E402
from dronebench.teammate_policy import ReferenceTrajectory  # noqa: E402


def test_bounded_feedforward_clips_and_rejects_nonfinite():
    assert np.allclose(bounded_feedforward([.1, 0, 0], .5), [.1, 0, 0])
    assert np.isclose(np.linalg.norm(bounded_feedforward([3, 4, 0], .5)), .5)
    with pytest.raises(ValueError):
        bounded_feedforward([np.nan, 0, 0], .5)


def test_reference_rates_on_constant_acceleration_and_yaw(tmp_path):
    t = np.arange(0, 2.001, .01)
    a, w = .2, .3
    x, vx = .5*a*t**2, a*t
    yaw = w*t
    rows = np.column_stack((t, x, 0*t, .9+0*t, vx, 0*t, 0*t, np.cos(yaw/2), 0*t, 0*t, np.sin(yaw/2)))
    path = tmp_path/'r.csv'
    np.savetxt(path, rows, delimiter=',', header='t,x,y,z,vx,vy,vz,qw,qx,qy,qz', comments='')
    ref = ReferenceTrajectory(path, time_scale=1)
    acc, yr = reference_rates(ref, 1.)
    assert np.allclose(acc, [a, 0, 0], atol=1e-6) and abs(yr-w) < 1e-6
