import numpy as np
import pytest
from dronebench.map_vio import MapVioCalibration,DeclaredHeadingAlignment
from dronebench.vision_odometry import OdometrySample


def sample(**overrides):
    values=dict(stamp_s=1.,clock_domain='test',frame_id='global',child_frame_id='imu',
        position=[0,0,0],orientation_xyzw=[0,0,0,1],linear_velocity=[0,2,0],angular_velocity=[0,0,2],
        pose_covariance=np.diag([.01]*3+[.04]*3),twist_covariance=np.diag([.01]*3+[.04]*3))
    values.update(overrides); return OdometrySample(**values)


def test_rotational_imu_velocity_and_full_covariance_lever_arm_are_compensated():
    T=np.eye(4);T[0,3]=1
    observation=MapVioCalibration(T,np.eye(3),[1,0,0]).body(sample())
    np.testing.assert_allclose(observation.position,[0,0,0])
    np.testing.assert_allclose(observation.velocity,[0,0,0])
    np.testing.assert_allclose(np.diag(observation.position_covariance),[.01,.05,.05])
    np.testing.assert_allclose(np.diag(observation.velocity_covariance),[.01,.05,.05])


def test_explicit_map_rotation_changes_position_and_velocity_in_the_same_frame():
    R=np.array([[0,-1,0],[1,0,0],[0,0,1.]])
    observation=MapVioCalibration(np.eye(4),R,[2,3,0]).body(sample(position=[1,0,0],linear_velocity=[1,0,0]))
    np.testing.assert_allclose(observation.position,[2,4,0])
    np.testing.assert_allclose(observation.velocity,[0,1,0])
    with pytest.raises(ValueError):MapVioCalibration(np.eye(4),R,[2,3,0]).body(sample(child_frame_id='base_link'))
    with pytest.raises(ValueError):MapVioCalibration(np.eye(4),np.zeros((3,3)),[0,0,0])


def test_independent_vio_yaw_initializations_align_to_declared_heading_without_zeroing_translation():
    for yaw in (0.,np.pi,-np.pi/2):
        aligner=DeclaredHeadingAlignment(np.eye(4),np.pi/2,[1,3])
        for t in np.linspace(1,1.6,30):
            row=sample(stamp_s=float(t),position=[.2,0,0],orientation_xyzw=[0,0,np.sin(yaw/2),np.cos(yaw/2)])
            R=aligner.observe(row)
        result=MapVioCalibration(np.eye(4),R,[2,3,0]).body(row)
        np.testing.assert_allclose(result.rotation,[[0,-1,0],[1,0,0],[0,0,1]],atol=1e-12)
        np.testing.assert_allclose(result.position,np.array([2,3,0])+R@np.array([.2,0,0]))
    with pytest.raises(ValueError):DeclaredHeadingAlignment(np.eye(4),0,[1,2]).observe(sample(stamp_s=3))


def test_declared_heading_refuses_changing_orientation_or_source_reset():
    aligner=DeclaredHeadingAlignment(np.eye(4),0,[1,3])
    aligner.observe(sample())
    with pytest.raises(ValueError):aligner.observe(sample())
    aligner=DeclaredHeadingAlignment(np.eye(4),0,[1,3])
    with pytest.raises(ValueError):
        for t in np.linspace(1,1.6,30):
            angle=(t-1)*.1
            aligner.observe(sample(stamp_s=float(t),orientation_xyzw=[0,0,np.sin(angle/2),np.cos(angle/2)]))
