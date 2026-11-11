"""Conservative live OpenVINS feature-count watchdog for course execution.

Consumes only OpenVINS' ordinary SLAM PointCloud2 output. It is a run inhibit
and stop trigger, not a position correction or estimator-quality certificate.
"""
from dataclasses import dataclass
from math import isfinite


@dataclass
class VioFeatureMonitor:
    min_slam_features: int = 8
    max_low_duration_s: float = 2.5
    startup_observation_s: float = 3.0
    startup_timeout_wall_s: float = 45.0
    max_source_age_s: float = 0.5
    max_arrival_age_s: float = 0.5
    # The subscriber's cached /clock can lag the publisher's stamp by exactly
    # one physics step (240 Hz) when both are produced in the same step:
    # observed 4.1667 ms in v35. Allow that one-step race only; any larger
    # lead is still a genuinely future-dated message and latches a fault.
    max_clock_lead_s: float = 1/240 + 1e-6

    def __post_init__(self):
        if type(self.min_slam_features) is not int or self.min_slam_features < 1:
            raise ValueError('positive SLAM feature threshold required')
        if any(not isfinite(v) or v <= 0 for v in
               (self.max_low_duration_s, self.startup_observation_s,
                self.startup_timeout_wall_s, self.max_source_age_s, self.max_arrival_age_s)):
            raise ValueError('positive finite VIO feature timing bounds required')
        self.first_source_s = None
        self.last_source_s = None
        self.last_progress_wall_s = None
        self.last_arrival_wall_s = None
        self.latest_count = None
        self.low_since_s = None
        self.adequate_since_s = None
        self.ever_ready = False
        self.fault = None
        self.samples = 0
        self.events = []

    def invalidate(self, reason, *, received_wall_s):
        if not self.fault:
            self.fault = str(reason)
            wall = float(received_wall_s) if type(received_wall_s) in (int, float) and isfinite(received_wall_s) else None
            self.events.append({'kind': 'fault', 'reason': self.fault,
                                'arrival_wall_s': wall})

    def observe(self, *, source_s, received_wall_s, feature_count):
        if (type(source_s) not in (int, float) or not isfinite(source_s) or source_s < 0
                or type(received_wall_s) not in (int, float) or not isfinite(received_wall_s)
                or type(feature_count) is not int or feature_count < 0):
            self.invalidate('invalid OpenVINS SLAM feature observation', received_wall_s=received_wall_s)
            return
        if self.last_source_s is not None and source_s < self.last_source_s:
            self.invalidate('OpenVINS SLAM feature source clock regressed', received_wall_s=received_wall_s)
            return
        if self.last_arrival_wall_s is not None and received_wall_s < self.last_arrival_wall_s:
            self.invalidate('OpenVINS SLAM feature arrival clock regressed', received_wall_s=received_wall_s)
            return
        if (self.last_progress_wall_s is not None
                and received_wall_s - self.last_progress_wall_s > self.max_arrival_age_s):
            self.invalidate('OpenVINS SLAM feature stream had an arrival gap', received_wall_s=received_wall_s)
            return
        if (self.last_source_s is not None
                and source_s - self.last_source_s > self.max_source_age_s):
            self.invalidate('OpenVINS SLAM feature stream had a source-time gap', received_wall_s=received_wall_s)
            return
        if self.last_source_s is not None and source_s == self.last_source_s:
            self.last_arrival_wall_s = received_wall_s
            return
        self.first_source_s = source_s if self.first_source_s is None else self.first_source_s
        self.last_source_s = source_s
        self.last_progress_wall_s = received_wall_s
        self.last_arrival_wall_s = received_wall_s
        self.latest_count = feature_count
        self.samples += 1
        if (self.low_since_s is not None
                and source_s - self.low_since_s > self.max_low_duration_s):
            self.invalidate('OpenVINS SLAM feature count stayed below threshold',
                            received_wall_s=received_wall_s)
            return
        if feature_count < self.min_slam_features:
            self.adequate_since_s = None
            if self.low_since_s is None:
                self.low_since_s = source_s
                self.events.append({'kind': 'low_features_started', 'source_s': source_s,
                                    'feature_count': feature_count})
        else:
            if self.adequate_since_s is None:
                self.adequate_since_s = source_s
            if self.low_since_s is not None:
                self.events.append({'kind': 'feature_count_recovered', 'source_s': source_s,
                                    'feature_count': feature_count,
                                    'low_duration_s': source_s - self.low_since_s})
                self.low_since_s = None

    def status(self, *, now_sim_s, now_wall_s):
        if self.fault:
            return {'state': 'FAULT', 'reason': self.fault}
        if self.last_source_s is None:
            return {'state': 'WAITING', 'reason': 'no OpenVINS SLAM feature cloud observed'}
        if (now_wall_s < self.last_arrival_wall_s
                or now_wall_s - self.last_arrival_wall_s > self.max_arrival_age_s
                or now_wall_s - self.last_progress_wall_s > self.max_arrival_age_s):
            self.invalidate('OpenVINS SLAM feature stream stale at arrival', received_wall_s=now_wall_s)
            return {'state': 'FAULT', 'reason': self.fault}
        source_age = now_sim_s - self.last_source_s
        # ROS time and message stamps are nanosecond values converted to
        # binary floats. Equal stamps can therefore subtract to a tiny
        # negative epsilon; tolerate that representation noise, not a real
        # future-dated message.
        if not 0 <= self.max_clock_lead_s <= 0.005:
            raise ValueError('clock lead tolerance must stay within one physics step')
        if source_age < -self.max_clock_lead_s:
            self.invalidate('OpenVINS SLAM feature stamp is ahead of the ROS clock', received_wall_s=now_wall_s)
            return {'state': 'FAULT', 'reason': self.fault, 'source_age_s': source_age,
                    'source_s': self.last_source_s, 'now_sim_s': now_sim_s}
        source_age = max(0.0, source_age)
        if source_age > self.max_source_age_s:
            self.invalidate(f'OpenVINS SLAM source age {source_age:.3f}s exceeds {self.max_source_age_s:.3f}s',
                            received_wall_s=now_wall_s)
            return {'state': 'FAULT', 'reason': self.fault, 'source_age_s': source_age,
                    'source_s': self.last_source_s, 'now_sim_s': now_sim_s}
        if self.low_since_s is not None and now_sim_s - self.low_since_s > self.max_low_duration_s:
            self.invalidate('OpenVINS SLAM feature count stayed below threshold', received_wall_s=now_wall_s)
            return {'state': 'FAULT', 'reason': self.fault}
        currently_adequate = self.latest_count >= self.min_slam_features
        warmed = (currently_adequate and self.adequate_since_s is not None
                  and now_sim_s - self.adequate_since_s >= self.startup_observation_s)
        if warmed:
            self.ever_ready = True
        state = 'READY' if warmed else ('DEGRADED' if self.ever_ready else 'WAITING')
        return {'state': state, 'reason': None if state == 'READY' else 'waiting for adequate fresh SLAM tracks',
                'latest_feature_count': self.latest_count, 'low_since_s': self.low_since_s,
                'samples': self.samples, 'adequate_since_s': self.adequate_since_s}

    def report(self):
        return {'kind': 'openvins_slam_feature_watchdog_not_estimator_certificate',
                'thresholds': {'minimum_slam_features': self.min_slam_features,
                               'maximum_continuous_low_s': self.max_low_duration_s,
                               'startup_observation_s': self.startup_observation_s,
                               'startup_timeout_wall_s': self.startup_timeout_wall_s,
                               'maximum_source_age_s': self.max_source_age_s,
                               'maximum_arrival_age_s': self.max_arrival_age_s},
                'samples': self.samples, 'latest_feature_count': self.latest_count,
                'ever_ready': self.ever_ready, 'fault': self.fault,
                'events': self.events}


def finite_xyz_count(cloud):
    """Count usable XYZ samples from OpenVINS' PointCloud2 feature output."""
    from sensor_msgs_py import point_cloud2
    fields = {field.name for field in cloud.fields}
    if not {'x', 'y', 'z'} <= fields:
        raise ValueError('SLAM feature PointCloud2 lacks XYZ fields')
    points = point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
    count = 0
    for point in points:
        values = point.tolist() if hasattr(point, 'tolist') else point
        if all(isfinite(float(value)) for value in values):
            count += 1
    return count

