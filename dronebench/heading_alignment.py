"""One-time VIO-world heading calibration from qualified PX4 attitude.

No simulator pose or fabricated north reference is used. The caller supplies
fresh, synchronized pre-EV compass/attitude data, then freezes this calibration.
This aligns coordinate axes; it does not qualify position accuracy or flight.
"""
from math import atan2, cos, sin
import numpy as np

from dronebench.vision_odometry import ENU_TO_NED, FLU_TO_FRD, quaternion_matrix


class HeadingAlignment:
    def __init__(self, required_span_s=1., min_samples=20):
        self.required_span_s, self.min_samples = required_span_s, min_samples
        self.samples = []
        self.result = None
        self.metadata = None

    def observe(self, *, vio_stamp_s, vio_xyzw, attitude_stamp_s, attitude_wxyz,
                now_s, flags, flags_stamp_s):
        if self.result is not None:
            return self.result.copy()
        times = np.asarray([vio_stamp_s, attitude_stamp_s, now_s, flags_stamp_s], dtype=float)
        if not np.isfinite(times).all() or np.min(times) < 0:
            raise ValueError('heading timestamps must be finite nonnegative shared-clock values')
        if (abs(vio_stamp_s-attitude_stamp_s) > .02 or now_s < max(vio_stamp_s, attitude_stamp_s, flags_stamp_s)
                or now_s-min(vio_stamp_s, attitude_stamp_s) > .1 or now_s-flags_stamp_s > .5):
            raise ValueError('heading inputs not synchronized/fresh in shared simulation clock')
        if (flags.get('cs_yaw_align') is not True or flags.get('cs_tilt_align') is not True
                or flags.get('cs_ev_yaw') is not False
                or not (flags.get('cs_mag_hdg') is True or flags.get('cs_mag_3d') is True)
                or flags.get('cs_mag_field_disturbed') is not False or flags.get('cs_mag_fault') is not False):
            self.samples.clear()
            raise ValueError('require healthy compass-aligned PX4 before any EV yaw fusion')
        if self.samples and vio_stamp_s <= self.samples[-1][0]:
            raise ValueError('heading source time must advance')
        r_vio = quaternion_matrix(vio_xyzw)
        w, x, y, z = attitude_wxyz
        r_px4 = quaternion_matrix((x, y, z, w))
        alignment = ENU_TO_NED.T @ r_px4 @ FLU_TO_FRD @ r_vio.T
        if not np.allclose(alignment[2], [0, 0, 1], atol=.02, rtol=0):
            self.samples.clear()
            raise ValueError('VIO/PX4 tilt disagreement exceeds heading-calibration envelope')
        yaw = atan2(alignment[1, 0], alignment[0, 0])
        self.samples.append((vio_stamp_s, yaw))
        if len(self.samples) < self.min_samples or self.samples[-1][0]-self.samples[0][0] < self.required_span_s:
            return None
        angles = np.asarray([s[1] for s in self.samples])
        mean = atan2(np.sin(angles).mean(), np.cos(angles).mean())
        residual = np.arctan2(np.sin(angles-mean), np.cos(angles-mean))
        if np.max(np.abs(residual)) > .01:
            self.samples = self.samples[-1:]
            raise ValueError('heading calibration has not stabilized')
        self.result = np.array([[cos(mean), -sin(mean), 0], [sin(mean), cos(mean), 0], [0, 0, 1.]])
        self.metadata = {'kind': 'one_time_pre_EV_compass_attitude_heading_alignment',
                         'samples': len(self.samples), 'source_span_s': self.samples[-1][0]-self.samples[0][0],
                         'world_to_enu_yaw_rad': mean, 'max_residual_rad': float(np.max(np.abs(residual))),
                         'world_to_enu': self.result.tolist(), 'truth_input': False}
        return self.result.copy()
