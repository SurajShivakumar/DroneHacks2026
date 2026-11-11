"""cuVSLAM adapter pure helpers (no ROS/GPU)."""
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import cuvslam_vo_node as n  # noqa: E402


def test_quaternion_round_trip_and_rotvec():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.normal(size=4); q /= np.linalg.norm(q)
        R = n.R_from_quat_xyzw(q)
        q2 = n.quat_xyzw_from_R(R)
        assert np.allclose(n.R_from_quat_xyzw(q2), R, atol=1e-9)
    th = .3
    Rz = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
    assert np.allclose(n.rotvec_from_R(Rz), [0, 0, th])


def test_gyro_gate_trips_only_on_sustained_disagreement():
    g = n.GyroGate(tolerance=.35, window_s=.5, hold_s=.4)
    t = 0.
    for _ in range(200):          # 1 s of gyro at 0.2 rad/s
        g.add_gyro(t, [0, 0, .2]); t += .005
    assert g.check(t, .25)        # consistent
    def feed_until(t_end):
        nonlocal t
        while t < t_end:
            g.add_gyro(t, [0, 0, .2]); t += .005
    t0 = t
    feed_until(t0+.1); assert g.check(t, 1.0)       # disagreement starts, not yet held
    feed_until(t0+.3); assert g.check(t, 1.0)
    feed_until(t0+.55); assert not g.check(t, 1.0) and g.tripped   # held > 0.4 s
    feed_until(t0+.6); assert not g.check(t, .2)    # latched for the run


def test_gyro_gate_waits_for_data():
    g = n.GyroGate()
    assert g.check(1.0, 5.0)      # no gyro yet: cannot judge
