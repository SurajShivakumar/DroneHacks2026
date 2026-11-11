from math import cos, pi, sin
import numpy as np
import pytest

from dronebench.heading_alignment import HeadingAlignment


def flags():
    return dict(cs_yaw_align=True, cs_tilt_align=True, cs_ev_yaw=False,
                cs_mag_hdg=True, cs_mag_3d=False, cs_mag_field_disturbed=False, cs_mag_fault=False)


def sample(aligner, t, **kw):
    args = dict(vio_stamp_s=t, vio_xyzw=(0, 0, 0, 1), attitude_stamp_s=t,
                attitude_wxyz=(1, 0, 0, 0), now_s=t, flags=flags(), flags_stamp_s=t)
    args.update(kw)
    return aligner.observe(**args)


def test_north_reference_is_measured_and_frozen():
    a = HeadingAlignment()
    for i in range(32):
        result = sample(a, 1+i*.04)
    # PX4 FRD body aligned north, VIO FLU body along its x: ENU north=+Y.
    np.testing.assert_allclose(result @ [1, 0, 0], [0, 1, 0], atol=1e-8)
    before = result.copy()
    result = sample(a, 3, attitude_wxyz=(0, 0, 0, 1))
    np.testing.assert_array_equal(before, result)
    assert a.metadata['truth_input'] is False


def test_arbitrary_vio_heading_is_not_silently_north():
    a = HeadingAlignment()
    for i in range(32):
        result = sample(a, 1+i*.04, vio_xyzw=(0, 0, sin(pi/4), cos(pi/4)))
    np.testing.assert_allclose(result, np.eye(3), atol=1e-8)


@pytest.mark.parametrize('change', [dict(cs_yaw_align=False), dict(cs_ev_yaw=True),
                                    dict(cs_mag_field_disturbed=True), dict(cs_mag_fault=True),
                                    dict(cs_mag_hdg=False)])
def test_unqualified_or_circular_heading_rejected(change):
    f = flags(); f.update(change)
    with pytest.raises(ValueError, match='healthy compass'):
        sample(HeadingAlignment(), 1, flags=f)


def test_stale_or_unsynchronized_heading_rejected():
    with pytest.raises(ValueError, match='synchronized'):
        sample(HeadingAlignment(), 1, attitude_stamp_s=.9)
    with pytest.raises(ValueError, match='synchronized'):
        sample(HeadingAlignment(), 1, flags_stamp_s=.1)


def test_tilt_disagreement_and_invalid_quaternion_rejected():
    with pytest.raises(ValueError, match='tilt'):
        sample(HeadingAlignment(), 1, attitude_wxyz=(cos(.1), sin(.1), 0, 0))
    with pytest.raises(ValueError, match='unit norm'):
        sample(HeadingAlignment(), 1, vio_xyzw=(0, 0, 0, 2))
