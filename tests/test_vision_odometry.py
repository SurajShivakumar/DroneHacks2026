from dataclasses import replace
from math import cos, sin, pi
from types import SimpleNamespace

import numpy as np
import pytest

from dronebench.vision_odometry import (ENU_TO_NED, FLU_TO_FRD, OdometrySample,
                                        VisionConverter, quaternion_matrix)


def sample(**changes):
    base = OdometrySample(1., 'ros_sim', 'global', 'imu', (1., 2., 3.), (0., 0., 0., 1.),
                          (4., 5., 6.), (.1, .2, .3), np.diag([1., 2., 3., 4., 5., 6.]),
                          np.diag([7., 8., 9., 10., 11., 12.]))
    return replace(base, **changes)


def convert(value=None, **configuration):
    return VisionConverter(clock_domain='ros_sim', **configuration).convert(
        value or sample(), now_s=1.05, now_clock_domain='ros_sim')


def rotation_from_output(result):
    w, x, y, z = result.q_wxyz
    return quaternion_matrix((x, y, z, w))


def test_arbitrary_heading_world_frd_is_default_not_north():
    result = convert()
    assert result.pose_frame == 'FRD' and result.velocity_frame == 'BODY_FRD'
    assert result.position == (1., -2., -3.)
    assert result.velocity == (4., -5., -6.)
    assert result.angular_velocity == (.1, -.2, -.3)
    assert result.q_wxyz == (1., 0., 0., 0.)
    assert result.timestamp_us == 1050000 and result.timestamp_sample_us == 1000000
    assert result.position_variance == (1., 2., 3.)
    assert result.orientation_variance == (4., 5., 6.)


@pytest.mark.parametrize('angle', [pi/2, -pi/2, pi, pi-1e-10])
def test_heading_rotation_uses_both_world_and_body_basis(angle):
    value = sample(orientation_xyzw=(0., 0., sin(angle/2), cos(angle/2)))
    result = convert(value)
    expected = FLU_TO_FRD @ quaternion_matrix(value.orientation_xyzw) @ FLU_TO_FRD
    np.testing.assert_allclose(rotation_from_output(result), expected, atol=1e-12)


@pytest.mark.parametrize('axis', [(1., 0., 0.), (0., 1., 0.), (0., 0., 1.)])
def test_half_turn_quaternion_conversion_is_stable(axis):
    value = sample(orientation_xyzw=(*axis, 0.))
    result = convert(value)
    np.testing.assert_allclose(rotation_from_output(result),
                               FLU_TO_FRD @ quaternion_matrix(value.orientation_xyzw) @ FLU_TO_FRD,
                               atol=1e-12)


def test_ned_requires_explicit_heading_alignment():
    with pytest.raises(ValueError, match='explicit calibrated'):
        VisionConverter(clock_domain='ros_sim', pose_frame='NED')
    result = convert(pose_frame='NED', world_to_enu=np.eye(3))
    assert result.position == (2., 1., -3.)
    assert result.position_variance == (2., 1., 3.)
    assert result.orientation_variance == (4., 5., 6.)  # Rotation tangent is body-local, not world.
    np.testing.assert_allclose(rotation_from_output(result), ENU_TO_NED @ FLU_TO_FRD, atol=1e-12)
    # ENU +90deg yaw body points north -> level identity NED/FRD orientation.
    result = convert(sample(orientation_xyzw=(0., 0., sin(pi/4), cos(pi/4))),
                     pose_frame='NED', world_to_enu=np.eye(3))
    np.testing.assert_allclose(rotation_from_output(result), np.eye(3), atol=1e-12)


def test_full_correlated_covariance_uses_correct_mixed_basis():
    lower = np.eye(6)
    lower[3, 0], lower[4, 1], lower[2, 0] = .2, -.3, .4
    covariance = lower @ lower.T
    result = convert(sample(pose_covariance=covariance), pose_frame='NED', world_to_enu=np.eye(3))
    jacobian = np.zeros((6, 6))
    jacobian[:3, :3], jacobian[3:, 3:] = ENU_TO_NED, FLU_TO_FRD
    np.testing.assert_allclose(result.pose_covariance, jacobian @ covariance @ jacobian.T)
    assert result.pose_covariance[1][3] == .2


def test_explicit_coincident_mount_rotation_is_applied_to_pose_twist_and_covariance():
    mount = np.eye(4)
    mount[:3, :3] = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]
    result = convert(body_from_imu=mount)
    body_map = FLU_TO_FRD @ mount[:3, :3]
    np.testing.assert_allclose(result.velocity, body_map @ np.array([4., 5., 6.]))
    np.testing.assert_allclose(rotation_from_output(result), FLU_TO_FRD @ mount[:3, :3].T @ FLU_TO_FRD, atol=1e-12)
    assert result.orientation_variance == (5., 4., 6.)
    assert result.velocity_variance == (8., 7., 9.)


def test_rotating_offset_imu_is_transformed_to_stationary_body_com():
    mount=np.eye(4); mount[:3,3]=[.1,0.,0.]
    # Body COM is at zero. IMU at +X moves +Y under a +Z angular rate.
    result=convert(sample(position=(.1,0.,0.),linear_velocity=(0.,.2,0.),angular_velocity=(0.,0.,2.)),body_from_imu=mount)
    np.testing.assert_allclose(result.position,[0,0,0],atol=1e-12)
    np.testing.assert_allclose(result.velocity,[0,0,0],atol=1e-12)
    # Rotation and angular-rate uncertainty must reach position/velocity.
    assert result.position_variance[1] > 2.
    assert result.velocity_variance[1] > 8.


@pytest.mark.parametrize('change', [
    {'position': (float('nan'), 0., 0.)},
    {'orientation_xyzw': (0., 0., 0., 0.)},
    {'orientation_xyzw': (0., 0., 0., 2.)},
    {'linear_velocity': (1., float('inf'), 0.)},
    {'angular_velocity': (0., 0.)},
    {'frame_id': 'map_reset'},
    {'child_frame_id': 'camera'},
    {'stamp_s': float('nan')},
    {'clock_domain': 'wall'},
])
def test_invalid_inputs_produce_no_state_advance(change):
    converter = VisionConverter(clock_domain='ros_sim')
    with pytest.raises(ValueError):
        converter.convert(sample(**change), now_s=1.05, now_clock_domain='ros_sim')
    assert converter.last_stamp_s is None


@pytest.mark.parametrize('kind', ['shape', 'nan', 'negative_diagonal', 'asymmetric', 'indefinite'])
def test_invalid_covariance_is_rejected_without_clamping(kind):
    covariance = np.eye(6)
    if kind == 'shape':
        covariance = np.eye(3)
    elif kind == 'nan':
        covariance[0, 0] = float('nan')
    elif kind == 'negative_diagonal':
        covariance[0, 0] = -.01
    elif kind == 'asymmetric':
        covariance[0, 1] = .1
    else:
        covariance[0, 1] = covariance[1, 0] = 2.
    with pytest.raises(ValueError):
        convert(sample(pose_covariance=covariance))


def test_same_clock_freshness_and_source_monotonicity():
    converter = VisionConverter(clock_domain='ros_sim')
    converter.convert(sample(), now_s=1.05, now_clock_domain='ros_sim')
    for value, now, clock in [(sample(), 1.06, 'ros_sim'),
                              (sample(stamp_s=.99), 1.06, 'ros_sim'),
                              (sample(stamp_s=1.01), 1.3, 'ros_sim'),
                              (sample(stamp_s=1.2), 1.1, 'ros_sim'),
                              (sample(stamp_s=1.01), 1.06, 'system'),
                              (sample(stamp_s=1.01), 1.04, 'ros_sim')]:
        with pytest.raises(ValueError):
            converter.convert(value, now_s=now, now_clock_domain=clock)
    assert converter.last_stamp_s == 1.
    converter.convert(sample(stamp_s=1.1), now_s=1.15, now_clock_domain='ros_sim')


def test_duck_typed_ros_message_extraction():
    vec = lambda x, y, z: SimpleNamespace(x=x, y=y, z=z)
    pose = SimpleNamespace(position=vec(1., 2., 3.), orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    twist = SimpleNamespace(linear=vec(4., 5., 6.), angular=vec(.1, .2, .3))
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=10000000), frame_id='global'),
                              child_frame_id='imu', pose=SimpleNamespace(pose=pose, covariance=np.eye(6).ravel()),
                              twist=SimpleNamespace(twist=twist, covariance=np.eye(6).ravel()))
    value = OdometrySample.from_ros(message, clock_domain='ros_sim')
    assert value.stamp_s == 1.01
    assert convert(value).position == (1., -2., -3.)
    message.header.stamp.nanosec = 1000000000
    with pytest.raises(ValueError):
        OdometrySample.from_ros(message, clock_domain='ros_sim')


def test_small_indefinite_covariance_is_not_mistaken_for_zero_roundoff():
    covariance = np.zeros((6, 6))
    covariance[0, 1] = covariance[1, 0] = 1e-12
    with pytest.raises(ValueError, match='semidefinite'):
        convert(sample(pose_covariance=covariance))


def test_age_boundary_and_timestamp_size():
    converter = VisionConverter(clock_domain='ros_sim')
    converter.convert(sample(), now_s=1.1, now_clock_domain='ros_sim')
    with pytest.raises(ValueError, match='uint64'):
        VisionConverter(clock_domain='ros_sim').convert(sample(stamp_s=1e20), now_s=1e20, now_clock_domain='ros_sim')
