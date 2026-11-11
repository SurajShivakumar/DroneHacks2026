"""Disarmed mode-dependent preflight diagnostic. Pure decisions, no ROS I/O.

This reuses HoverGuard.health only; it never calls the hover mission state
machine. The only recommendations are a constant-current-position heartbeat
stream and one Offboard mode request. No motor or flight commands are available.
"""
from dataclasses import dataclass
from math import isfinite, sqrt

from dronebench.flight_guard import HoverGuard


def native_commander_ack_matches(message, *, expected_commands, target_system, target_component, request_sent):
    """Match a native PX4 v1.16 Commander acknowledgment, not an external ACK.

    Pinned PX4 6ea3539157ca358c70a515878b77077af7d4611d:
    Commander.cpp:answer_command zero-initializes vehicle_command_ack_s and
    sets command/result, destination=the request source IDs and timestamp.
    It leaves ACK.from_external false. mavlink_receiver.cpp:
    handle_message_command_ack sets from_external true for externally RECEIVED
    acknowledgments. This field does not echo VehicleCommand.from_external.
    Freshness, accepted result and actual resulting state remain separate gates.
    """
    if (request_sent is not True or type(target_system) is not int or not 1 <= target_system <= 255
            or type(target_component) is not int or not 1 <= target_component <= 65535):
        return False
    return (getattr(message, 'command', None) in expected_commands
            and getattr(message, 'target_system', None) == target_system
            and getattr(message, 'target_component', None) == target_component
            and getattr(message, 'from_external', None) is False)


@dataclass(frozen=True)
class ProbeAction:
    kind: str
    position: tuple | None = None
    yaw: float | None = None

    def __post_init__(self):
        if self.kind not in ('stream', 'mode_offboard'):
            raise ValueError('action outside disarmed diagnostic scope')


class DisarmedOffboardProbe:
    max_wall_s = 120.
    startup_window_s = 30.
    offboard_window_s = 30.
    max_movement_m = .05
    terminal = {'COMPLETE', 'FAULT'}

    def __init__(self, *, started_wall, probe_mode=False):
        self.telemetry = HoverGuard(started_wall=started_wall, execute=False)
        self.started_wall, self.probe_mode = started_wall, probe_mode
        self.state, self.reason = 'WAITING', 'waiting for actual telemetry'
        self.startup_since = self.preflight_since = None
        self.startup_completed_wall = None
        self.last_sim = self.last_wall = None
        self.origin = self.origin_resets = self.yaw = None
        self.first_stream_sim = self.first_stream_wall = None
        self.stream_count = 0
        self.mode_request = self.mode_ack = None
        self.ack_accepted = False
        self.events = []
        self.waiting_ids = None
        self.last_health_failure = None

    def observe(self, name, *, stamp_s, received_wall, data):
        self.telemetry.observe(name, stamp_s=stamp_s, received_wall=received_wall, data=data)

    def acknowledge(self, *, result, stamp_s, received_wall):
        if self.mode_request is not None and result in ('accepted', 'rejected', 'in_progress'):
            if isfinite(stamp_s) and isfinite(received_wall):
                self.mode_ack = (result, stamp_s, received_wall)

    def status(self):
        return self.telemetry.streams['status'].data or {}

    def land(self):
        return self.telemetry.streams['land'].data or {}

    def position(self):
        return self.telemetry.streams['position'].data or {}

    def publication_error(self, *, now_sim, now_wall):
        error = self.telemetry.health(now_sim=now_sim, now_wall=now_wall)
        if error:
            return error
        status, land, position = self.status(), self.land(), self.position()
        if status.get('armed') is not False or status.get('disarmed') is not True:
            return 'vehicle is not explicitly disarmed; stop immediately'
        if land.get('landed') is not True:
            return 'vehicle is not landed; stop immediately'
        if self.origin is not None:
            if sqrt(sum((position[key]-origin)**2 for key, origin in zip(('x', 'y', 'z'), self.origin))) > self.max_movement_m:
                return 'disarmed position envelope moved'
            if tuple(position.get(key) for key in HoverGuard.reset_fields) != self.origin_resets:
                return 'estimator reference/reset counters changed'
        if self.state == 'PRESTREAM' and status.get('auto_loiter') is not True:
            return 'original AUTO_LOITER mode changed before the request'
        if self.state == 'WAIT_MODE' and not (status.get('auto_loiter') or status.get('offboard')):
            return 'unexpected mode during Offboard request'
        if self.state == 'OFFBOARD_OBSERVE' and status.get('offboard') is not True:
            return 'Offboard authority lost'
        return None

    def streamed(self, *, now_sim, now_wall):
        if self.first_stream_sim is None:
            self.first_stream_sim, self.first_stream_wall = now_sim, now_wall
        self.stream_count += 1

    def _fault(self, reason):
        self.state, self.reason = 'FAULT', reason
        return []

    def step(self, *, now_sim, now_wall):
        if self.state in self.terminal:
            return []
        if not isfinite(now_sim) or now_sim < 0 or not isfinite(now_wall):
            return self._fault('invalid current clock')
        if ((self.last_wall is not None and now_wall < self.last_wall)
                or (self.last_sim is not None and now_sim < self.last_sim)):
            return self._fault('current clock regressed')
        if self.state != 'WAITING' and self.last_wall is not None:
            if now_wall-self.last_wall > .25 or now_sim-self.last_sim > .25:
                return self._fault('control heartbeat cadence lost')
        self.last_sim, self.last_wall = now_sim, now_wall
        if now_wall-self.started_wall >= self.max_wall_s:
            return self._fault('120 wall-second diagnostic deadline')
        status, land, position = self.status(), self.land(), self.position()
        if status.get('armed') is True or status.get('disarmed') is False:
            return self._fault('armed state observed; diagnostic stopped')
        health = self.telemetry.health(now_sim=now_sim, now_wall=now_wall)
        if self.state == 'WAITING':
            invalid = next((f'{name}: {stream.invalid}' for name, stream in self.telemetry.streams.items() if stream.invalid), None)
            if invalid:
                return self._fault(invalid)  # A source reset/regression is not a transient age issue.
            identity = (status.get('system_id'), status.get('component_id'))
            if self.waiting_ids is not None and identity != self.waiting_ids:
                return self._fault('PX4 identity changed during idle observation')
            if health is not None:
                self.last_health_failure = {'kind': 'startup_health_window_reset', 'reason': health,
                    'now_sim_s': now_sim, 'now_wall_s': now_wall, 'previous_startup_since_wall_s': self.startup_since,
                    'stream_ages': {name: {'stamp_s': stream.stamp,
                        'source_age_sim_s': now_sim-stream.stamp if stream.stamp is not None else None,
                        'arrival_age_wall_s': now_wall-stream.received if stream.received is not None else None,
                        'progress_age_wall_s': now_wall-stream.progressed if stream.progressed is not None else None}
                        for name, stream in self.telemetry.streams.items()}}
                self.events.append(self.last_health_failure)
                self.startup_since, self.reason = None, health
                return []
            ready = health is None and status.get('disarmed') is True and land.get('landed') is True and status.get('auto_loiter') is True
            if not ready:
                if self.waiting_ids is not None:
                    return self._fault(health or 'landed/disarmed/original mode condition changed')
                self.reason = health or 'require original AUTO_LOITER, landed and disarmed'
                return []
            if self.startup_since is None:
                self.startup_since = now_wall
                self.waiting_ids = identity
            self.reason = 'observing original disarmed AUTO_LOITER health (preflight flag is recorded, not bypassed)'
            if now_wall-self.startup_since < self.startup_window_s or not self.probe_mode:
                return []
            self.origin = tuple(position[key] for key in ('x', 'y', 'z'))
            self.origin_resets = tuple(position.get(key) for key in HoverGuard.reset_fields)
            if any(value is None for value in self.origin_resets):
                return self._fault('missing reset counters')
            self.yaw = position['heading']
            self.startup_completed_wall = now_wall
            self.telemetry.target_ids = (status['system_id'], status['component_id'])
            self.state = 'PRESTREAM'
        error = self.publication_error(now_sim=now_sim, now_wall=now_wall)
        if error:
            return self._fault(error)
        if self.state == 'PRESTREAM':
            if (self.stream_count >= 22 and now_sim-self.first_stream_sim >= 1.1
                    and now_wall-self.first_stream_wall >= 1.1):
                self.mode_request = (now_sim, now_wall)
                self.state = 'WAIT_MODE'
                self.events.append({'kind': 'request_offboard_only', 'sim_s': now_sim, 'wall_s': now_wall})
                return [ProbeAction('stream', self.origin, self.yaw), ProbeAction('mode_offboard')]
        elif self.state == 'WAIT_MODE':
            if self.mode_ack is not None:
                result, stamp, received = self.mode_ack
                sent_sim, sent_wall = self.mode_request
                if (sent_sim <= stamp <= now_sim and sent_wall <= received <= now_wall
                        and now_sim-stamp <= .5 and now_wall-received <= .5):
                    if result == 'rejected':
                        return self._fault('Offboard mode request rejected')
                    if result == 'accepted' and not self.ack_accepted:
                        self.ack_accepted = True
                        self.events.append({'kind': 'mode_ack_accepted', 'source_sim_s': stamp, 'received_wall_s': received})
            if now_sim-self.mode_request[0] > 3 or now_wall-self.mode_request[1] > 10:
                return self._fault('Offboard mode ACK/status deadline')
            if self.ack_accepted and status.get('offboard') is True:
                self.state = 'OFFBOARD_OBSERVE'
                self.events.append({'kind': 'actual_offboard_disarmed', 'sim_s': now_sim, 'wall_s': now_wall})
        if self.state == 'OFFBOARD_OBSERVE':
            if status.get('preflight') is True:
                if self.preflight_since is None:
                    self.preflight_since = now_wall
                self.reason = 'observing native Offboard preflight checks while disarmed'
                if now_wall-self.preflight_since >= self.offboard_window_s:
                    self.state, self.reason = 'COMPLETE', None
                    self.events.append({'kind': 'native_offboard_preflight_30s', 'sim_s': now_sim, 'wall_s': now_wall})
                    return []
            else:
                self.preflight_since = None
                self.reason = 'native PX4 pre_flight_checks_pass remains false in Offboard'
        return [ProbeAction('stream', self.origin, self.yaw)]

    def report(self):
        wall = self.last_wall
        startup_end = self.startup_completed_wall if self.startup_completed_wall is not None else wall
        startup = max(0., startup_end-self.startup_since) if startup_end is not None and self.startup_since is not None else 0.
        preflight = max(0., wall-self.preflight_since) if wall is not None and self.preflight_since is not None else 0.
        return {'kind': 'disarmed_offboard_mode_preflight_diagnostic', 'state': self.state, 'reason': self.reason,
                'probe_mode': self.probe_mode, 'startup_window_s': startup,
                'startup_window_satisfied': startup >= self.startup_window_s,
                'offboard_preflight_window_s': preflight,
                'offboard_preflight_window_satisfied': self.state == 'COMPLETE',
                'mode_requested': self.mode_request is not None, 'mode_ack_accepted': self.ack_accepted,
                'heartbeat_samples_published': self.stream_count, 'constant_position_ned': self.origin,
                'last_checkpoint': {'now_sim_s': self.last_sim, 'now_wall_s': self.last_wall},
                'last_health_failure': self.last_health_failure,
                'max_movement_m': self.max_movement_m, 'events': self.events,
                'freshness_contract': HoverGuard.freshness,
                'telemetry': {name: {'last_fields': stream.data, 'stamp_s': stream.stamp,
                                     'advances': stream.advances, 'invalid': stream.invalid}
                              for name, stream in self.telemetry.streams.items()},
                'flight_ready_claimed': False, 'no_actuation_or_flight_command_supported': True}
