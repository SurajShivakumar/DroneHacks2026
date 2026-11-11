"""Pure OpenVINS odometry conversion, with no ROS I/O or readiness bypass.

Pinned input contract: gravity-up world, Hamilton xyzw IMU->world orientation,
twist in IMU-local FLU, pose covariance [position_world, rotation_IMU_local],
twist covariance [linear_IMU_local, angular_IMU_local]. This is not a generic
ROS covariance convention: verify every estimator adapter against its source.

Default output is world-fixed FRD with arbitrary heading (PX4 POSE_FRAME_FRD),
not north-aligned NED. NED needs an explicit calibrated source-world->ENU yaw
rotation. All enum values are symbolic; a ROS adapter maps installed definitions.
PX4 v1.16 contract: https://docs.px4.io/v1.16/en/msg_docs/VehicleOdometry

Calibrated IMU/body offsets transform position, angular-rate-induced velocity
and the full pose/twist covariance Jacobians. Extrinsic uncertainty is not
provided by this interface and must be qualified separately. PX4's variance arrays
lose correlations. Their availability does not establish estimator health,
observability, frame-reset continuity, qualification or permission to fly.
"""

from dataclasses import dataclass
from math import isfinite, sqrt

import numpy as np


FLU_TO_FRD = np.diag([1., -1., -1.])
ENU_TO_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])


def _scalar(value, name):
    if type(value) not in (int, float) or not isfinite(value) or value < 0:
        raise ValueError(f'{name} must be finite and nonnegative')
    return float(value)


def _array(value, shape, name):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f'{name} must be finite with shape {shape}')
    return result


def _rotation(value, name):
    result = _array(value, (3, 3), name)
    if not np.allclose(result.T @ result, np.eye(3), atol=1e-8, rtol=0) or not np.isclose(np.linalg.det(result), 1., atol=1e-8, rtol=0):
        raise ValueError(f'{name} must be a proper rotation')
    return result


def _covariance(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape == (36,):
        result = result.reshape(6, 6)
    result = _array(result, (6, 6), name)
    scale = max(float(np.abs(result).max()), np.finfo(float).tiny)
    tolerance = 100 * np.finfo(float).eps * scale
    if (not np.allclose(result, result.T, atol=tolerance, rtol=1e-7)
            or np.any(np.diag(result) < 0)
            or np.linalg.eigvalsh(result).min() < -tolerance):
        raise ValueError(f'{name} must be symmetric positive semidefinite with nonnegative diagonal')
    return result


def quaternion_matrix(xyzw):
    """Hamilton active body->world rotation; reject invalid rather than repair it."""
    q = _array(xyzw, (4,), 'orientation_xyzw')
    length = np.linalg.norm(q)
    if abs(length - 1.) > 1e-6:
        raise ValueError('orientation quaternion must have unit norm')
    x, y, z, w = q / length  # Remove only accepted floating-point unit roundoff.
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def _matrix_quaternion(rotation):
    """Proper rotation -> Hamilton wxyz, including near-180-degree rotations."""
    r = rotation
    trace = np.trace(r)
    if trace > 0:
        scale = 2 * sqrt(1 + trace)
        q = np.array([scale / 4, (r[2, 1]-r[1, 2])/scale,
                      (r[0, 2]-r[2, 0])/scale, (r[1, 0]-r[0, 1])/scale])
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i+1) % 3, (i+2) % 3
        scale = 2 * sqrt(1 + r[i, i] - r[j, j] - r[k, k])
        vector = np.zeros(3)
        vector[i] = scale / 4
        vector[j] = (r[j, i]+r[i, j]) / scale
        vector[k] = (r[k, i]+r[i, k]) / scale
        q = np.r_[(r[k, j]-r[j, k]) / scale, vector]
    return tuple(float(x) for x in (q if q[0] >= 0 else -q))


@dataclass(frozen=True)
class OdometrySample:
    stamp_s: float
    clock_domain: str
    frame_id: str
    child_frame_id: str
    position: object
    orientation_xyzw: object
    linear_velocity: object
    angular_velocity: object
    pose_covariance: object
    twist_covariance: object

    @classmethod
    def from_ros(cls, message, *, clock_domain):
        """Duck-typed nav_msgs/Odometry extraction; no ROS package import needed."""
        pose, twist = message.pose.pose, message.twist.twist
        xyz = lambda value: (value.x, value.y, value.z)
        q = pose.orientation
        stamp = message.header.stamp
        if type(stamp.sec) is not int or type(stamp.nanosec) is not int or not 0 <= stamp.nanosec < 1000000000:
            raise ValueError('invalid ROS timestamp fields')
        return cls(stamp.sec + stamp.nanosec / 1e9, clock_domain,
                   message.header.frame_id, message.child_frame_id,
                   xyz(pose.position), (q.x, q.y, q.z, q.w), xyz(twist.linear), xyz(twist.angular),
                   message.pose.covariance, message.twist.covariance)


@dataclass(frozen=True)
class VisionOdometry:
    timestamp_us: int
    timestamp_sample_us: int
    clock_domain: str
    pose_frame: str
    velocity_frame: str
    position: tuple
    q_wxyz: tuple
    velocity: tuple
    angular_velocity: tuple
    pose_covariance: tuple
    twist_covariance: tuple
    position_variance: tuple
    orientation_variance: tuple
    velocity_variance: tuple


class VisionConverter:
    """Validate and convert a single continuous, already-qualified estimator stream.

    Clock domains must be identical: this never estimates offsets between ROS,
    PX4 boot time or wall time. In shared-clock SITL the adapter can supply ROS
    simulation time; hardware needs a separately reviewed time synchronization.
    Source stamps and sample microseconds must advance. Rejected samples produce
    no output and do not advance internal state. After a source/frame/clock reset,
    establish a new reviewed stream and converter; do not silently relabel it.

    body_from_imu is a calibrated 4x4 transform IMU FLU->body FLU. If omitted,
    caller asserts the two frames coincide exactly. Translation is the IMU
    origin expressed in body coordinates; the output refers to the body origin.
    """

    def __init__(self, *, clock_domain, max_age_s=.1, world_frame='global', imu_frame='imu',
                 pose_frame='FRD', world_to_enu=None, body_from_imu=None):
        if not isinstance(clock_domain, str) or not clock_domain:
            raise ValueError('explicit clock_domain required')
        if not isinstance(world_frame, str) or not world_frame or not isinstance(imu_frame, str) or not imu_frame:
            raise ValueError('explicit nonempty input frames required')
        self.max_age_s = _scalar(max_age_s, 'max_age_s')
        if self.max_age_s == 0:
            raise ValueError('max_age_s must be positive')
        if pose_frame == 'FRD':
            if world_to_enu is not None:
                raise ValueError('world_to_enu is only valid for calibrated NED output')
            self.world_map = FLU_TO_FRD.copy()
        elif pose_frame == 'NED':
            if world_to_enu is None:
                raise ValueError('NED needs explicit calibrated source-world->ENU rotation')
            alignment = _rotation(world_to_enu, 'world_to_enu')
            if not np.allclose(alignment[2], (0, 0, 1), atol=1e-8, rtol=0):
                raise ValueError('source world must already be gravity-up; heading alignment is yaw only')
            self.world_map = ENU_TO_NED @ alignment
        else:
            raise ValueError('pose_frame must be FRD or calibrated NED')
        transform = np.eye(4) if body_from_imu is None else _array(body_from_imu, (4, 4), 'body_from_imu')
        if not np.allclose(transform[3], (0, 0, 0, 1), atol=1e-10, rtol=0):
            raise ValueError('body_from_imu must be rigid')
        self.body_map = FLU_TO_FRD @ _rotation(transform[:3, :3], 'body_from_imu rotation')
        self.imu_to_body = transform[:3, :3].copy()
        self.imu_position_in_body = transform[:3, 3].copy()
        self.clock_domain, self.pose_frame = clock_domain, pose_frame
        self.world_frame, self.imu_frame = world_frame, imu_frame
        self.last_stamp_s = self.last_now_s = self.last_sample_us = None

    def convert(self, sample, *, now_s, now_clock_domain):
        if not isinstance(sample, OdometrySample):
            raise ValueError('OdometrySample required')
        if sample.clock_domain != self.clock_domain or now_clock_domain != self.clock_domain:
            raise ValueError('clock domains differ; no implicit clock conversion allowed')
        if sample.frame_id != self.world_frame or sample.child_frame_id != self.imu_frame:
            raise ValueError('input frame contract changed')
        stamp, now = _scalar(sample.stamp_s, 'stamp_s'), _scalar(now_s, 'now_s')
        time_roundoff = 2 * float(np.spacing(max(now, stamp)))
        if now < stamp or now - stamp > self.max_age_s + time_roundoff:
            raise ValueError('future or stale odometry')
        if max(stamp, now) > (2**64 - 1) / 1e6:
            raise ValueError('timestamp cannot fit PX4 uint64 microseconds')
        sample_us = int(stamp * 1e6)
        if (self.last_stamp_s is not None and (stamp <= self.last_stamp_s or sample_us <= self.last_sample_us)
                or self.last_now_s is not None and now < self.last_now_s):
            raise ValueError('source or receive clock did not advance consistently')
        p = _array(sample.position, (3,), 'position')
        orientation = quaternion_matrix(sample.orientation_xyzw)
        v = _array(sample.linear_velocity, (3,), 'linear_velocity')
        w = _array(sample.angular_velocity, (3,), 'angular_velocity')
        pose_cov = _covariance(sample.pose_covariance, 'pose_covariance')
        twist_cov = _covariance(sample.twist_covariance, 'twist_covariance')
        pose_jacobian = np.zeros((6, 6))
        pose_jacobian[:3, :3], pose_jacobian[3:, 3:] = self.world_map, self.body_map
        def skew(value):
            x,y,z=value
            return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
        arm_in_imu=self.imu_to_body.T @ self.imu_position_in_body
        pose_jacobian[:3,3:]=self.world_map @ orientation @ skew(arm_in_imu)
        twist_jacobian = np.zeros((6, 6))
        twist_jacobian[:3, :3] = twist_jacobian[3:, 3:] = self.body_map
        twist_jacobian[:3,3:]=FLU_TO_FRD @ skew(self.imu_position_in_body) @ self.imu_to_body
        converted_pose_cov = pose_jacobian @ pose_cov @ pose_jacobian.T
        converted_twist_cov = twist_jacobian @ twist_cov @ twist_jacobian.T
        _array(converted_pose_cov, (6, 6), 'converted pose covariance')
        _array(converted_twist_cov, (6, 6), 'converted twist covariance')
        rotation = self.world_map @ orientation @ self.imu_to_body.T @ FLU_TO_FRD
        vector = lambda values: tuple(float(value) for value in values)
        result = VisionOdometry(int(now * 1e6), sample_us, self.clock_domain,
                                self.pose_frame, 'BODY_FRD', vector(_array(self.world_map @ (p-orientation @ arm_in_imu), (3,), 'converted position')),
                                _matrix_quaternion(rotation), vector(self.body_map @ v+twist_jacobian[:3,3:] @ w), vector(self.body_map @ w),
                                tuple(vector(row) for row in converted_pose_cov),
                                tuple(vector(row) for row in converted_twist_cov),
                                vector(np.diag(converted_pose_cov)[:3]), vector(np.diag(converted_pose_cov)[3:]),
                                vector(np.diag(converted_twist_cov)[:3]))
        self.last_stamp_s, self.last_now_s, self.last_sample_us = stamp, now, sample_us
        return result
