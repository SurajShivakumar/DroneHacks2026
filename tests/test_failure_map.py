import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import failure_map  # noqa: E402
import prepare_fast_reference as pfr  # noqa: E402


def test_projection_is_monotone_along_a_loop():
    th = np.linspace(0, 1.5*np.pi, 400)
    pts = np.column_stack([np.cos(th), np.sin(th)])
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    track = pts[::4]+np.random.default_rng(0).normal(0, .01, (100, 2))
    out = failure_map.project(track, pts, s)
    assert np.all(np.diff(out) > -.05)
    assert abs(out[-1]-s[-1]) < .1


def test_classify():
    assert failure_map.classify({'state': 'COMPLETE'}) == 'complete'
    assert failure_map.classify({'state': 'ABORT', 'reason': 'VIO feature quality lost'}) == 'vio'
    assert failure_map.classify({'state': 'ABORT', 'reason': 'trajectory tracking deviation'}) == 'tracking'
    assert failure_map.classify({'state': 'ABORT', 'reason': 'x', 'vio_fault': 'low'}) == 'vio'


def _bin(**kw):
    b = {'surv_speeds': [], 'reliable_speeds': [], 'fail_speeds': []}
    b.update(kw)
    return b


def test_caps_ignore_slow_deaths_and_respect_reliable_floor():
    bins = [_bin(surv_speeds=[.3, .5]),                      # proven 0.5
            _bin(surv_speeds=[.2, .4], fail_speeds=[.05]),   # hover death: no cap
            _bin(surv_speeds=[.2, .6], reliable_speeds=[.25], fail_speeds=[.3]),  # 0.24 -> floor 0.25
            _bin()]                                          # nobody reached: carry last
    caps = failure_map.bin_caps(bins, bin_m=.25, fail_spread_m=0., fail_factor=.8, min_fail_speed=.15, min_cap=.12)
    assert caps.tolist() == [.5, .4, .25, .25]


def test_cap_spread_reaches_neighbours():
    bins = [_bin(surv_speeds=[.5]) for _ in range(5)]
    bins[2]['fail_speeds'] = [.4]
    caps = failure_map.bin_caps(bins, bin_m=.25, fail_spread_m=.25, fail_factor=.5, min_fail_speed=.15, min_cap=.12)
    assert caps.tolist() == [.5, .2, .2, .2, .5]


def test_start_turn_profiles_end_at_the_target_and_respect_omega():
    for profile, expect in (('smoothstep', 1.875*1.54/.25), ('trapezoid', 1.54/.25+1.5)):
        dur, ang = pfr.start_turn(-1.54, .25, profile, 1.5)
        assert abs(dur-expect) < 1e-9
        tau = np.arange(0, dur+1e-9, .01)
        a = np.array([ang(t) for t in tau])
        assert abs(a[0]) < 1e-9 and abs(a[-1]+1.54) < 1e-6
        assert np.abs(np.diff(a)/.01).max() <= .25+1e-6
    dur, ang = pfr.start_turn(.1, .25, 'trapezoid', 1.5)   # small turn -> triangle
    assert abs(ang(dur)-.1) < 1e-9
