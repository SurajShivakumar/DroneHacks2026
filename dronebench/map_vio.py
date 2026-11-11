"""Explicit gravity-up OpenVINS world -> published-map body observations.

Uses the pinned OpenVINS covariance/twist contract, not generic ROS odometry.
No automatic world alignment, reset correction, truth or publishing. Extrinsic
uncertainty and pose/twist cross-correlation are not supplied by this contract.
"""
from dataclasses import dataclass
import numpy as np
from dronebench.vision_odometry import OdometrySample,quaternion_matrix,_covariance,_rotation,_array


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


@dataclass(frozen=True)
class MapBodyObservation:
    source_s: float
    position: np.ndarray
    rotation: np.ndarray
    velocity: np.ndarray
    position_covariance: np.ndarray
    velocity_covariance: np.ndarray


class MapVioCalibration:
    def __init__(self,body_from_imu,map_from_vio_rotation,imu_origin_map):
        self.T=_array(body_from_imu,(4,4),'body_from_imu').copy()
        _rotation(self.T[:3,:3],'body_from_imu rotation')
        if not np.allclose(self.T[3],[0,0,0,1],atol=1e-10):raise ValueError('homogeneous calibrated extrinsics required')
        self.align=_rotation(map_from_vio_rotation,'map_from_vio rotation').copy()
        self.origin=_array(imu_origin_map,(3,),'IMU origin in map').copy()

    def body(self,sample):
        if not isinstance(sample,OdometrySample) or sample.frame_id!='global' or sample.child_frame_id!='imu':
            raise ValueError('pinned OpenVINS global/imu observation required')
        if not np.isfinite(sample.stamp_s) or sample.stamp_s<=0:raise ValueError('positive finite source stamp required')
        T=self.T; Ri=self.align@quaternion_matrix(sample.orientation_xyzw); R=Ri@T[:3,:3].T
        p=self.origin+self.align@_array(sample.position,(3,),'VIO position')-R@T[:3,3]
        angular=T[:3,:3]@_array(sample.angular_velocity,(3,),'IMU angular rate')
        v=Ri@_array(sample.linear_velocity,(3,),'IMU velocity')-R@np.cross(angular,T[:3,3])
        Jp=np.c_[self.align,Ri@skew(T[:3,:3].T@T[:3,3])]
        Jv=np.c_[Ri,R@skew(T[:3,3])@T[:3,:3]]
        Cp=Jp@_covariance(sample.pose_covariance,'VIO pose covariance')@Jp.T
        Cv=Jv@_covariance(sample.twist_covariance,'VIO twist covariance')@Jv.T
        return MapBodyObservation(sample.stamp_s,p,R,v,Cp,Cv)


class DeclaredHeadingAlignment:
    """Diagnostic calibration against a reviewed physical initial heading.

    The caller must establish that heading remains valid through the declared
    source window. Translation during that window is NOT zeroed. This is not
    flight-start qualification or a substitute for a real heading measurement.
    OpenVINS gravity initialization may choose different yaw axes each run.
    """
    def __init__(self,body_from_imu,declared_yaw,source_window):
        self.T=_array(body_from_imu,(4,4),'body_from_imu').copy()
        _rotation(self.T[:3,:3],'body_from_imu rotation')
        self.window=_array(source_window,(2,),'declared heading-valid source window')
        if self.window[0]<0 or self.window[0]>=self.window[1] or not np.isfinite(declared_yaw):
            raise ValueError('finite heading and increasing nonnegative declared window required')
        self.yaw=declared_yaw;self.samples=[];self.result=None

    def observe(self,sample):
        if self.result is not None:return self.result.copy()
        if not isinstance(sample,OdometrySample) or sample.frame_id!='global' or sample.child_frame_id!='imu':
            raise ValueError('pinned OpenVINS global/imu sample required')
        t=sample.stamp_s
        if not self.window[0]<=t<=self.window[1]:raise ValueError('initial heading no longer declared valid; new reviewed calibration required')
        if self.samples and t<=self.samples[-1][0]:raise ValueError('heading source reset/nonadvancing')
        R=quaternion_matrix(sample.orientation_xyzw)@self.T[:3,:3].T
        if np.arccos(np.clip(R[2,2],-1,1))>.03:raise ValueError('initial heading calibration exceeds level-tilt envelope')
        self.samples.append((t,self.yaw-np.arctan2(R[1,0],R[0,0])))
        if len(self.samples)<20 or t-self.samples[0][0]<.5:return None
        a=np.array([v for _,v in self.samples]);mean=np.arctan2(np.sin(a).mean(),np.cos(a).mean())
        if np.max(np.abs(np.arctan2(np.sin(a-mean),np.cos(a-mean))))>.01:
            raise ValueError('initial heading changed during declared calibration window')
        c,s=np.cos(mean),np.sin(mean);self.result=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        return self.result.copy()
