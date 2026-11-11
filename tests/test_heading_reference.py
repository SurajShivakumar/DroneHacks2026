"""Offline heading-planner and truth-evaluator regressions (no simulator)."""
from math import pi
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import prepare_heading_reference as h  # noqa: E402
import evaluate_flight_truth as ev  # noqa: E402
from dronebench.gate_audit import audit  # noqa: E402


class FlatScorer:
    """Prefers +x; lets build() run without published geometry."""
    def score(self, position, yaw):
        return float(np.cos(yaw))


def straight_reference():
    # rest, move +x 1 m, rest, move +y 1 m, rest
    rows = []
    t = 0.
    for _ in range(20):
        rows.append([t, 0, 0, .9, 0, 0, 0]); t += .01
    for x in np.linspace(.01, 1, 100):
        rows.append([t, x, 0, .9, .1, 0, 0]); t += .01
    for _ in range(20):
        rows.append([t, 1, 0, .9, 0, 0, 0]); t += .01
    for y in np.linspace(.01, 1, 100):
        rows.append([t, 1, y, .9, 0, .1, 0]); t += .01
    for _ in range(20):
        rows.append([t, 1, 1, .9, 0, 0, 0]); t += .01
    return np.array(rows, float)


def test_build_keeps_every_position_and_turns_only_at_rest():
    samples = straight_reference()
    chosen = [{'start_index': 19, 'end_index': 120, 'yaw_rad': 0.},
              {'start_index': 139, 'end_index': 240, 'yaw_rad': pi/2}]
    rows, turns = h.build(samples, chosen, FlatScorer(), start_yaw=pi/2)
    assert np.all(np.diff(rows[:, 0]) > 0)
    assert np.diff(rows[:, 0]).max() <= .0201
    # every source position appears in the output, in order
    out = rows[:, 1:4]
    for p in samples[:, 1:4]:
        assert np.min(np.linalg.norm(out-p, axis=1)) < 1e-12
    assert len(turns) == 2
    # yaw only changes while stationary
    moving = np.linalg.norm(rows[:, 4:7], axis=1) > 0
    yaw_change = np.abs(np.diff(rows[:, 7])) > 1e-12
    assert not np.any(yaw_change & moving[1:] & moving[:-1])
    # final approach into a turn rest is slowed
    approach = rows[(rows[:, 1] > .7) & (rows[:, 1] < .99) & (rows[:, 2] == 0)]
    assert np.abs(approach[:, 4]).max() <= .1/h.APPROACH_SLOW_FACTOR+1e-9


def test_tilt_aware_gate_audit_matches_strict_audit_for_vertical_poses():
    t = np.arange(0, 6, .01)
    x = -1.5+.5*t
    poses = np.column_stack((t, x, .05*np.ones_like(t), np.ones_like(t), np.ones_like(t),
                             np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)))
    gates = [dict(name='g', center=[0., 0., 1.], normal=[1., 0., 0.], half_width=.4, half_height=.35)]
    strict = audit(poses, gates, radius=.28, half_height=.13)
    tilt = ev.tilt_aware_gate_audit(poses, gates, radius=.28, half_height=.13)
    assert strict['ordered_planes_cleared'] and tilt['ordered_planes_cleared']
    narrow = [dict(gates[0], half_width=.3)]
    assert not audit(poses, narrow, radius=.28, half_height=.13)['ordered_planes_cleared']
    assert not ev.tilt_aware_gate_audit(poses, narrow, radius=.28, half_height=.13)['ordered_planes_cleared']
