"""Reuse the pinned teammate inference core and its saved reference trajectory.

No ROS, simulator API, truth pose, arming, or reset commands. All course-frame
alignment comes from reviewed physical placement plus actual estimator/range
measurements. Known gate coordinates are the published course prior.
"""
from dataclasses import dataclass
from math import atan2, cos, isfinite, sin
from pathlib import Path
import importlib.util

import numpy as np


def rotation_wxyz(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-4:
        raise ValueError('finite unit FRD-to-NED quaternion required')
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def floor_height_from_range(distance, q_ned_frd, offset_frd, axis_frd):
    """Vertical AGL of body origin from a floor return and calibrated sensor TF.

    Caller must establish that the beam sees the floor at the reviewed launch
    mark; a range over an obstacle does not establish floor height.
    """
    rotation = rotation_wxyz(q_ned_frd)
    axis, offset = np.asarray(axis_frd, float), np.asarray(offset_frd, float)
    if (axis.shape != (3,) or offset.shape != (3,) or not np.isfinite(offset).all()
            or not np.isfinite(axis).all() or abs(np.linalg.norm(axis)-1) > 1e-6
            or not isfinite(distance) or not .05 <= distance <= 4.):
        raise ValueError('valid floor range and calibrated sensor extrinsic required')
    down = (rotation @ axis)[2]
    if down < .95:
        raise ValueError('range beam not sufficiently downward for launch calibration')
    height = distance*down+(rotation @ offset)[2]
    if not .05 <= height <= 1.:
        raise ValueError('launch AGL outside calibration envelope')
    return float(height)


@dataclass(frozen=True)
class CourseFrame:
    ned_to_course: np.ndarray
    origin_in_course: np.ndarray
    north_heading_in_course: float

    @classmethod
    def from_launch(cls, *, position_ned, heading_ned, launch_xy, body_agl,
                    physical_heading_in_course, placement_reviewed=False):
        values = np.asarray([*position_ned, heading_ned, *launch_xy, body_agl, physical_heading_in_course])
        if not placement_reviewed or values.shape != (8,) or not np.isfinite(values).all() or body_agl <= 0:
            raise ValueError('reviewed physical launch alignment and actual pose/range required')
        # NED yaw is clockwise viewed from above; course yaw is CCW/Z-up.
        # Under M below, course_heading = north_heading - NED_heading.
        north = physical_heading_in_course+heading_ned
        matrix = np.array([[cos(north), sin(north), 0.], [sin(north), -cos(north), 0.], [0., 0., -1.]])
        origin = np.array([*launch_xy, body_agl])-matrix @ np.asarray(position_ned)
        return cls(matrix, origin, north)

    def to_course(self, position_ned):
        return self.ned_to_course @ np.asarray(position_ned)+self.origin_in_course

    def to_ned(self, position_course):
        return self.ned_to_course.T @ (np.asarray(position_course)-self.origin_in_course)


class ReferenceTrajectory:
    """Position tracking of the learned path, explicitly distinct from PPO.

    Stretching reference time is allowed here; scaling the direct PPO actions
    would be a different, unvalidated controller. Reverse reference is an
    unqualified RETURN candidate until separately collision/flight checked.
    """
    def __init__(self, csv_path, *, time_scale=4., reverse=False):
        self.samples = np.loadtxt(csv_path, delimiter=',', skiprows=1)
        if (self.samples.ndim != 2 or self.samples.shape[0] < 2 or self.samples.shape[1] not in (11,15)
                or not np.isfinite(self.samples).all() or self.samples[0, 0] != 0
                or np.any(np.diff(self.samples[:, 0]) <= 0)
                or not isfinite(time_scale) or time_scale < 1):
            raise ValueError('finite chronological teammate trajectory and time_scale >=1 required')
        self.scale, self.reverse = time_scale, bool(reverse)
        self.duration_s = float(self.samples[-1, 0]*time_scale)
        q=self.samples[:,7:11]
        if not np.allclose(np.linalg.norm(q,axis=1),1,atol=1e-3):
            raise ValueError('unit finite reference quaternions required')
        self.yaws=np.unwrap(np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2]**2+q[:,3]**2)))

    def sample(self, elapsed_s):
        if not isfinite(elapsed_s) or elapsed_s < 0:
            raise ValueError('finite nonnegative reference time required')
        t = min(elapsed_s/self.scale, self.samples[-1, 0])
        if self.reverse:
            t = self.samples[-1, 0]-t
        point = np.array([np.interp(t, self.samples[:, 0], self.samples[:, i]) for i in (1, 2, 3)])
        velocity = np.array([np.interp(t, self.samples[:, 0], self.samples[:, i]) for i in (4, 5, 6)])/self.scale
        if self.reverse:
            velocity = -velocity
        return point, velocity

    def nearest(self, position, *, around_s, behind_s=1., ahead_s=2.):
        """Project measured position onto a LOCAL reference interval.

        Restricting the search prevents jumping to a different route segment
        where the course later passes nearby. This reads only sensor position
        and the reviewed reference, never simulator state.
        """
        position=np.asarray(position,dtype=float)
        if (position.shape!=(3,) or not np.isfinite(position).all()
                or not all(isfinite(v) and v>=0 for v in (around_s,behind_s,ahead_s))):
            raise ValueError('finite local path projection inputs required')
        low=max(0.,min(self.duration_s,around_s-behind_s))
        high=min(self.duration_s,around_s+ahead_s)
        original_times=self.samples[:,0]*self.scale
        original_points=self.samples[:,1:4]
        if self.reverse:
            original_times=self.duration_s-original_times[::-1];original_points=original_points[::-1]
        times=np.r_[low,original_times[(original_times>low)&(original_times<high)],high]
        points=np.stack([np.interp(times,original_times,original_points[:,i]) for i in range(3)],axis=-1)
        segments=points[1:]-points[:-1]; length2=np.sum(segments*segments,axis=1)
        fractions=np.clip(np.sum((position-points[:-1])*segments,axis=1)/np.maximum(length2,1e-20),0,1)
        projected=points[:-1]+fractions[:,None]*segments
        distances=np.linalg.norm(projected-position,axis=1); index=int(np.argmin(distances))
        phase=times[index]+fractions[index]*(times[index+1]-times[index])
        return float(phase),projected[index],float(distances[index])

    def yaw(self, elapsed_s):
        if not isfinite(elapsed_s) or elapsed_s<0:raise ValueError('finite nonnegative yaw time required')
        t=min(elapsed_s/self.scale,self.samples[-1,0])
        if self.reverse:t=self.samples[-1,0]-t
        return float(np.interp(t,self.samples[:,0],self.yaws))


class TrackingDeviation(ValueError):
    """Fresh navigation state exceeded the independently screened margin."""


class TrackingClock:
    """Pause the reference when tracking lags or the camera needs to turn."""
    def __init__(self, *, max_error_m=.25, pause_error_m=.12, rate_slew_per_s=None, max_yaw_rate=.5, lag_pause_m=None):
        if not all(isfinite(v) for v in (max_error_m,pause_error_m)) or not 0<pause_error_m<max_error_m<=.25:
            raise ValueError('reviewed positive tracking limits required')
        self.max_error_m,self.pause_error_m=max_error_m,pause_error_m
        # Along-track lag (aircraft behind the moving reference) is not a
        # clearance quantity; the cross-track pause/abort above are. None keeps
        # the original behaviour (lag throttles at the cross-track pause).
        if lag_pause_m is not None and (not isfinite(lag_pause_m) or not pause_error_m<=lag_pause_m<=.25):
            raise ValueError('lag pause must be within [pause_error_m, 0.25] m')
        self.lag_pause_m=pause_error_m if lag_pause_m is None else lag_pause_m
        if rate_slew_per_s is not None and (not isfinite(rate_slew_per_s) or not 0<rate_slew_per_s<=1):
            raise ValueError('bounded positive reference rate slew required')
        self.rate_slew_per_s=rate_slew_per_s
        if not isfinite(max_yaw_rate) or not 0<max_yaw_rate<=.5:
            raise ValueError('positive bounded camera yaw rate required')
        self.max_yaw_rate=max_yaw_rate
        self.speed_fraction=0. if rate_slew_per_s is not None else 1.
        self.elapsed_s=0.
        self.last_sim=None
        self.yaw_command=None

    def step(self, *, now_sim, error_m, desired_yaw, actual_yaw, progress_floor_s=0., lag_error_m=0.):
        if not all(isfinite(v) for v in (now_sim,error_m,desired_yaw,actual_yaw)) or error_m<0:
            raise ValueError('finite tracking state required')
        if error_m>self.max_error_m:
            raise TrackingDeviation('trajectory tracking deviation exceeds screened corridor')
        if not all(isfinite(v) and v>=0 for v in (progress_floor_s,lag_error_m)):
            raise ValueError('finite measured path progress and lag required')
        if self.last_sim is not None and not 0<=now_sim-self.last_sim<=.25:
            raise ValueError('tracking clock regressed or control cadence lost')
        dt=0. if self.last_sim is None else now_sim-self.last_sim
        self.last_sim=now_sim
        if self.yaw_command is None: self.yaw_command=actual_yaw
        difference=atan2(sin(desired_yaw-self.yaw_command),cos(desired_yaw-self.yaw_command))
        self.yaw_command+=max(-self.max_yaw_rate*dt,min(self.max_yaw_rate*dt,difference))
        heading_error=abs(atan2(sin(desired_yaw-actual_yaw),cos(desired_yaw-actual_yaw)))
        if self.rate_slew_per_s is None:
            self.speed_fraction=1. if error_m<self.pause_error_m and heading_error<.15 else 0.
        else:
            tracking_rate=max(0.,min(1.,(self.pause_error_m-error_m)/(.6*self.pause_error_m),
                                      (self.lag_pause_m-lag_error_m)/(.6*self.lag_pause_m)))
            heading_rate=max(0.,min(1.,(.15-heading_error)/.1))
            target_rate=min(tracking_rate,heading_rate)
            step=self.rate_slew_per_s*dt
            self.speed_fraction+=max(-step,min(step,target_rate-self.speed_fraction))
        self.elapsed_s=max(self.elapsed_s+dt*self.speed_fraction,progress_floor_s)
        return self.elapsed_s,self.yaw_command


class TeammatePolicy:
    """Thin wrapper: execute original PolicyRunner, validate its inputs/outputs."""
    def __init__(self, repo, checkpoint, frame):
        repo = Path(repo).resolve()
        spec = importlib.util.spec_from_file_location('dronehacks_teammate_runner', repo/'deploy/runner.py')
        module = importlib.util.module_from_spec(spec)
        import sys
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        self.core = module.PolicyRunner(str(checkpoint), {
            'px4_origin_in_course': frame.origin_in_course.tolist(),
            'px4_north_heading_in_course': frame.north_heading_in_course,
            'fence_margin': .15, 'ceiling_slack': 0., 'max_tilt_deg': 55.,
            'hold_on_missed_gate': True, 'thrust_min': .05, 'thrust_max': .95,
        }, str(repo/'config/course_v3.yaml'))
        self.frame = frame

    def align(self, frame):
        """Freeze a fresh launch calibration before first control publication."""
        self.frame = frame
        self.core.M = frame.ned_to_course.copy()
        self.core.origin = frame.origin_in_course.copy()
        self.core.reset()

    def reset(self):
        self.core.reset()

    def step(self, position_ned, q_ned_frd, velocity_ned, angular_velocity_frd, hover_thrust):
        rotation_wxyz(q_ned_frd)
        for value in (position_ned, velocity_ned, angular_velocity_frd):
            if np.asarray(value).shape != (3,) or not np.isfinite(value).all():
                raise ValueError('finite correctly framed policy state required')
        if not isfinite(hover_thrust) or not .1 < hover_thrust < .9:
            raise ValueError('fresh valid actual PX4 hover-thrust estimate required')
        result = self.core.step(position_ned, q_ned_frd, velocity_ned, angular_velocity_frd, hover_thrust)
        if (not np.isfinite(result.rates_frd).all() or not isfinite(result.thrust)
                or result.rates_frd.shape != (3,) or np.any(abs(result.rates_frd) > self.core.max_rates+1e-6)
                or not .05 <= result.thrust <= .95):
            raise ValueError('policy output outside reviewed action contract')
        return result
