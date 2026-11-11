"""Competitor-node helpers (no ROS)."""
from math import pi
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import skybeta_team_nav as n  # noqa: E402


def test_anchor_round_trip_and_launch_alignment():
    # odom started at (1,2,0.1) facing odom yaw 0.3; launch mark is course (-2.5, 0.9, 0.08) facing +y
    A = n.Anchor((1., 2., .1), .3, (-2.5, .9, .08), pi/2)
    assert np.allclose(A.to_course((1., 2., .1)), (-2.5, .9, .08))
    # moving 1 m along odom heading (yaw 0.3) is moving 1 m along course +y
    fwd = np.array([np.cos(.3), np.sin(.3), 0.])
    assert np.allclose(A.to_course(np.array((1., 2., .1))+fwd), (-2.5, 1.9, .08), atol=1e-9)
    c = np.array([.4, -1.2, .9])
    assert np.allclose(A.to_course(A.to_odom(c)), c)
    assert abs(n.wrap(A.yaw_to_odom(pi/2)-.3)) < 1e-12


def test_clock_runs_full_rate_on_track_and_pauses_off_track():
    c = n.Clock(pause_m=.05, lag_pause_m=.25, slew=.5)
    for _ in range(100):
        c.step(.05, cross_m=.0, lag_m=.0)
    assert abs(c.rate-1.) < 1e-9 and c.t > 3.
    t0 = c.t
    for _ in range(60):
        c.step(.05, cross_m=.06, lag_m=.0)       # beyond pause: rate slews to 0
    assert c.rate == 0. and c.t-t0 < 1.1
    assert c.step(.05, .06, 0., floor_t=c.t+5) == c.t   # progress floor still applies
