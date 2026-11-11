"""Pure bounded hover qualification decisions. No ROS I/O or flight commands.

The caller must use a reviewed shared simulation clock, one command owner and
matching PX4 messages. Approval alone does not establish estimator readiness.
An interruption or loss of this process is not evidence of landing.
"""
from dataclasses import dataclass
from math import hypot, isfinite, pi


@dataclass(frozen=True)
class Action:
    kind: str
    position: tuple | None = None
    yaw: float | None = None
    velocity: tuple | None = None   # optional NED feedforward for 'stream'


@dataclass
class Stream:
    stamp: float | None = None
    received: float | None = None
    progressed: float | None = None
    advances: int = 0
    invalid: str | None = None
    data: dict | None = None

    def observe(self, stamp, received, data):
        if not isfinite(stamp) or stamp < 0 or not isfinite(received):
            self.invalid = 'invalid telemetry timestamp'
            return
        if self.received is not None and received < self.received:
            self.invalid = 'arrival clock regressed'
        if self.stamp is not None and stamp < self.stamp:
            self.invalid = 'source clock regressed'
        if self.stamp is None or stamp > self.stamp:
            self.advances += self.stamp is not None
            self.progressed = received
        self.stamp, self.received, self.data = stamp, received, dict(data)


class HoverGuard:
    """0.5 m rise at 0.25 m/s, stable 2 s hover, one Land, observe disarm.

    Bounds: 0.2 m XY drift, relative altitude [-0.1,0.65] m; 120 wall seconds
    including preflight and 30 simulation seconds after control begins. Thirty
    continuous wall seconds of valid advancing telemetry precede any publisher.
    Actions are recommendations to the reviewed ROS adapter, never side effects.
    An accepted command ACK and fresh resulting mode/state are both required.
    """
    terminal = {'COMPLETE', 'FAULT', 'ABORT_LANDED'}
    height_m = .5
    climb_mps = .25
    hold_s = 2.
    max_xy_m = .2
    max_wall_s = 120.
    max_sim_s = 30.
    max_age_s = .5
    observe_wall_s = 30.
    reset_fields = ('xy_reset_counter', 'z_reset_counter', 'vxy_reset_counter', 'vz_reset_counter', 'heading_reset_counter')
    # Reviewed from actual native output cadence: position high-rate, status
    # about 2 Hz simulation time, land about 1 Hz. These bounds do not change
    # PX4 estimator checks. Frozen source progression has its own wall deadline.
    freshness = {'position': {'source_sim_s': .1, 'arrival_wall_s': .5, 'progress_wall_s': .5},
                 'status': {'source_sim_s': 1., 'arrival_wall_s': 2., 'progress_wall_s': 2.},
                 'land': {'source_sim_s': 2., 'arrival_wall_s': 3., 'progress_wall_s': 3.}}

    def __init__(self, *, started_wall, execute=False, approval_reference=None):
        if not isfinite(started_wall):
            raise ValueError('finite monotonic wall start required')
        if execute and (not isinstance(approval_reference, str) or not approval_reference.strip()):
            raise ValueError('execution requires a recorded team approval reference')
        self.execute, self.approval_reference = execute, approval_reference
        self.started_wall = started_wall
        self.state, self.reason = 'WAITING', None
        self.streams = {name: Stream() for name in ('status', 'position', 'land')}
        self.events, self.pending, self.acks, self.validated_acks = [], {}, {}, {}
        self.origin = self.origin_resets = self.yaw = self.target_ids = None
        self.ready_since = self.control_sim = self.climb_sim = self.hold_since = None
        self.last_tick_wall = self.last_tick_sim = None
        self.first_stream_wall = self.first_stream_sim = self.last_stream_wall = None
        self.stream_count = 0
        self.owns = False
        self.land_requested = False
        self.land_acknowledged = self.land_mode_observed = False
        self.abort_reason = None

    def observe(self, name, *, stamp_s, received_wall, data):
        if name not in self.streams:
            raise ValueError('unknown telemetry stream')
        self.streams[name].observe(stamp_s, received_wall, data)

    def acknowledge(self, name, *, result, stamp_s, received_wall):
        """Caller filters ACK target IDs against its actual command source IDs."""
        if name in self.pending and result in ('accepted', 'rejected', 'in_progress'):
            if isfinite(stamp_s) and isfinite(received_wall):
                self.acks[name] = (result, stamp_s, received_wall)

    def health(self, *, now_sim, now_wall):
        for name, stream in self.streams.items():
            budget = self.freshness[name]
            if stream.invalid:
                return f'{name}: {stream.invalid}'
            if stream.data is None or stream.advances < 1:
                return f'{name}: need actual advancing telemetry'
            if (now_wall < stream.received or now_wall - stream.received > budget['arrival_wall_s']
                    or now_wall - stream.progressed > budget['progress_wall_s']):
                return f'{name}: stale arrival/source progression'
            if now_sim < stream.stamp or now_sim - stream.stamp > budget['source_sim_s']:
                return f'{name}: source time not fresh in shared simulation clock'
        status = self.streams['status'].data
        position = self.streams['position'].data
        if status.get('failsafe') is not False:
            return 'PX4 failsafe or missing failsafe report'
        if status.get('armed') not in (True, False) or status.get('disarmed') not in (True, False) or status.get('armed') == status.get('disarmed'):
            return 'arming state is unknown or inconsistent'
        if any(type(status.get(key)) is not int or not 1 <= status[key] <= 255 for key in ('system_id', 'component_id')):
            return 'invalid or missing PX4 command target IDs'
        if self.target_ids is not None and (status['system_id'], status['component_id']) != self.target_ids:
            return 'PX4 identity changed'
        if any(position.get(key) is not True for key in ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid', 'heading_good_for_control')):
            return 'position, velocity or heading invalid'
        if any(type(position.get(key)) not in (int, float) or not isfinite(position[key])
               for key in ('x', 'y', 'z', 'vx', 'vy', 'vz', 'heading')):
            return 'nonfinite or missing position/velocity/heading'
        if not -pi <= position['heading'] <= pi:
            return 'heading outside PX4 contract'
        if self.streams['land'].data.get('freefall') is not False:
            return 'freefall or missing freefall report'
        return None

    def _status(self):
        return self.streams['status'].data or {}

    def _request(self, kind, now_sim, now_wall):
        self.pending[kind] = (now_sim, now_wall)
        self.events.append({'kind': 'request_' + kind, 'sim_s': now_sim, 'wall_s': now_wall})
        return Action(kind)

    def _ack(self, kind, now_sim, now_wall):
        item = self.acks.get(kind)
        if item is None:
            return self.validated_acks.get(kind)
        result, stamp, received = item
        sent_sim, sent_wall = self.pending[kind]
        if (stamp < sent_sim or received < sent_wall or received > now_wall
                or now_wall - received > self.max_age_s or stamp > now_sim
                or now_sim - stamp > self.max_age_s):
            return self.validated_acks.get(kind)
        if result in ('accepted', 'rejected') and kind not in self.validated_acks:
            self.validated_acks[kind] = result
            self.events.append({'kind': 'ack_' + kind, 'result': result, 'source_sim_s': stamp,
                                'received_wall_s': received})
        return result

    def _timed_out(self, kind, now_sim, now_wall):
        sim, wall = self.pending[kind]
        return now_sim - sim > 3 or now_wall - wall > 10

    def streamed(self, *, now_sim, now_wall):
        """Call only after heartbeat AND trajectory setpoint were published."""
        if self.first_stream_wall is None:
            self.first_stream_wall, self.first_stream_sim = now_wall, now_sim
        self.last_stream_wall = now_wall
        self.stream_count += 1

    def abort(self, reason, *, now_sim, now_wall):
        """Land at most once only if still fresh and the acknowledged owner."""
        self.abort_reason = self.reason = reason
        fresh = self.health(now_sim=now_sim, now_wall=now_wall) is None
        status = self._status()
        if fresh and self.owns and status.get('armed') and status.get('offboard') and not self.land_requested:
            self.land_requested = True
            self.owns = False
            self.state = 'LANDING'
            return [self._request('land', now_sim, now_wall)]
        self.state, self.owns = 'FAULT', False
        return []

    def step(self, *, now_sim, now_wall):
        if not isfinite(now_sim) or now_sim < 0 or not isfinite(now_wall):
            self.state, self.reason, self.owns = 'FAULT', 'invalid current clock', False
            return []
        if self.state in self.terminal:
            return []
        clock_regressed = ((self.last_tick_sim is not None and now_sim < self.last_tick_sim)
                           or (self.last_tick_wall is not None and now_wall < self.last_tick_wall))
        tick_stalled = (self.last_tick_wall is not None and self.state != 'WAITING'
                        and (now_wall - self.last_tick_wall > .25 or now_sim - self.last_tick_sim > .25))
        self.last_tick_sim, self.last_tick_wall = now_sim, now_wall
        if clock_regressed:
            self.state, self.reason, self.owns = 'FAULT', 'current clock regressed', False
            return []
        healthy = self.health(now_sim=now_sim, now_wall=now_wall)
        if self.state == 'WAITING':
            status, land = self._status(), self.streams['land'].data or {}
            ready = (healthy is None and status.get('disarmed') is True and status.get('armed') is False and status.get('offboard') is False
                     and status.get('land_mode') is False and status.get('preflight') is True
                     and status.get('startup_mode_allowed') is True and land.get('landed') is True)
            if not ready:
                self.ready_since = None
                self.reason = healthy or 'require landed/disarmed, preflight checks and idle supported mode'
            elif self.ready_since is None:
                self.ready_since = now_wall
                self.reason = 'observing continuous preflight telemetry'
            elif now_wall - self.ready_since >= self.observe_wall_s:
                self.reason = None
                if self.execute:
                    p = self.streams['position'].data
                    self.origin = (p['x'], p['y'], p['z'])
                    self.origin_resets = tuple(p.get(key) for key in self.reset_fields)
                    if any(value is None for value in self.origin_resets):
                        self.state, self.reason = 'FAULT', 'position reset counters missing'
                        return []
                    self.yaw = p['heading']
                    self.target_ids = (status['system_id'], status['component_id'])
                    self.state, self.control_sim = 'PRESTREAM', now_sim
            if now_wall - self.started_wall >= self.max_wall_s:
                self.state, self.reason = 'FAULT', 'wall deadline before flight readiness'
                return []
            if self.state == 'WAITING':
                return []
        elif healthy is not None:
            self.state, self.reason, self.owns = 'FAULT', healthy, False
            return []
        if self.state == 'LANDING':
            status, land = self._status(), self.streams['land'].data
            self.land_acknowledged |= self._ack('land', now_sim, now_wall) == 'accepted'
            self.land_mode_observed |= status.get('land_mode') is True
            if (land.get('landed') is True and status.get('disarmed') is True and status.get('armed') is False
                    and self.land_acknowledged and self.land_mode_observed):
                self.state = 'ABORT_LANDED' if self.abort_reason else 'COMPLETE'
                self.events.append({'kind': 'landed_disarmed', 'sim_s': now_sim, 'wall_s': now_wall})
            elif now_wall - self.started_wall >= self.max_wall_s or now_sim - self.control_sim >= self.max_sim_s:
                self.state, self.reason = 'FAULT', 'deadline awaiting landed and disarmed'
            elif self._ack('land', now_sim, now_wall) == 'rejected':
                self.state, self.reason = 'FAULT', 'Land rejected; control remains handed off'
            elif not status.get('offboard') and not status.get('land_mode'):
                self.state, self.reason = 'FAULT', 'authority changed after Land request'
            return []  # No heartbeat/setpoints after handing off to Land.
        status, p = self._status(), self.streams['position'].data
        if self.owns and (not status.get('armed') or not status.get('offboard')):
            self.state, self.reason, self.owns = 'FAULT', 'offboard/armed authority lost', False
            return []
        if tick_stalled:
            return self.abort('control loop stalled', now_sim=now_sim, now_wall=now_wall)
        if now_wall - self.started_wall >= self.max_wall_s or now_sim - self.control_sim >= self.max_sim_s:
            return self.abort('qualification deadline', now_sim=now_sim, now_wall=now_wall)
        if tuple(p.get(key) for key in self.reset_fields) != self.origin_resets:
            return self.abort('estimator reference reset', now_sim=now_sim, now_wall=now_wall)
        relative_height = self.origin[2] - p['z']
        if hypot(p['x'] - self.origin[0], p['y'] - self.origin[1]) > self.max_xy_m or not -.1 <= relative_height <= .65:
            return self.abort('position envelope exceeded', now_sim=now_sim, now_wall=now_wall)
        target = self.origin
        if self.state in ('PRESTREAM', 'WAIT_MODE') or self.state == 'WAIT_ARM' and not status.get('armed'):
            if status.get('preflight') is not True or self.streams['land'].data.get('landed') is not True:
                return self.abort('pre-arm checks or landed state changed', now_sim=now_sim, now_wall=now_wall)
        if self.state == 'PRESTREAM':
            if status.get('armed') or status.get('offboard'):
                return self.abort('unexpected authority before mode request', now_sim=now_sim, now_wall=now_wall)
            if (self.stream_count >= 22 and now_wall - self.first_stream_wall > 1
                    and now_sim - self.first_stream_sim > 1):
                self.state = 'WAIT_MODE'
                return [Action('stream', target, self.yaw), self._request('mode_offboard', now_sim, now_wall)]
        elif self.state == 'WAIT_MODE':
            if status.get('armed'):
                return self.abort('armed before this mission requested arming', now_sim=now_sim, now_wall=now_wall)
            ack = self._ack('mode_offboard', now_sim, now_wall)
            if ack == 'rejected' or self._timed_out('mode_offboard', now_sim, now_wall):
                return self.abort('Offboard mode rejected or timed out', now_sim=now_sim, now_wall=now_wall)
            if ack == 'accepted' and status.get('offboard'):
                self.state = 'WAIT_ARM'
                return [Action('stream', target, self.yaw), self._request('arm', now_sim, now_wall)]
        elif self.state == 'WAIT_ARM':
            if not status.get('offboard'):
                return self.abort('mode lost before arming acknowledgement', now_sim=now_sim, now_wall=now_wall)
            ack = self._ack('arm', now_sim, now_wall)
            if ack == 'rejected' or self._timed_out('arm', now_sim, now_wall):
                return self.abort('arming rejected or timed out', now_sim=now_sim, now_wall=now_wall)
            if ack == 'accepted' and status.get('armed'):
                self.state, self.owns, self.climb_sim = 'CLIMB', True, now_sim
                self.events.append({'kind': 'armed_offboard_observed', 'sim_s': now_sim, 'wall_s': now_wall})
        if self.state in ('CLIMB', 'HOLD'):
            rise = min(self.height_m, self.climb_mps * (now_sim - self.climb_sim))
            target = (self.origin[0], self.origin[1], self.origin[2] - rise)
            settled = rise >= self.height_m and abs(relative_height - self.height_m) <= .05 and abs(p['vz']) <= .1
            if settled:
                if self.hold_since is None:
                    self.hold_since, self.state = now_sim, 'HOLD'
                elif now_sim - self.hold_since >= self.hold_s:
                    self.land_requested, self.owns, self.state = True, False, 'LANDING'
                    return [self._request('land', now_sim, now_wall)]
            else:
                self.hold_since = None
        return [Action('stream', target, self.yaw)]

    def report(self, *, now_sim=None, now_wall=None):
        """Current read-only readiness window; never changes mission behavior."""
        now_sim = self.last_tick_sim if now_sim is None else now_sim
        now_wall = self.last_tick_wall if now_wall is None else now_wall
        status, land = self._status(), self.streams['land'].data or {}
        healthy = (now_sim is not None and now_wall is not None
                   and self.health(now_sim=now_sim, now_wall=now_wall) is None)
        current_ready = (self.state == 'WAITING' and healthy and status.get('armed') is False
                         and status.get('disarmed') is True and status.get('offboard') is False
                         and status.get('land_mode') is False and status.get('preflight') is True
                         and status.get('startup_mode_allowed') is True and land.get('landed') is True)
        ready_window = max(0., now_wall-self.ready_since) if current_ready and self.ready_since is not None else 0.
        return {'state': self.state, 'reason': self.reason, 'execute': self.execute,
                'ready_window_s': ready_window,
                'preflight_window_satisfied': current_ready and self.reason is None and ready_window >= self.observe_wall_s,
                'approval_reference': self.approval_reference, 'land_requested': self.land_requested,
                'land_acknowledged': self.land_acknowledged, 'land_mode_observed': self.land_mode_observed,
                'events': self.events, 'heartbeat_samples_published': self.stream_count,
                'freshness_contract': self.freshness,
                'plan': {'height_m': self.height_m, 'climb_mps': self.climb_mps, 'hold_sim_s': self.hold_s,
                         'max_xy_m': self.max_xy_m, 'max_wall_s': self.max_wall_s, 'max_sim_s': self.max_sim_s},
                'streams': {name: {'stamp_s': stream.stamp, 'advances': stream.advances,
                                    'invalid': stream.invalid, 'last_fields': stream.data}
                            for name, stream in self.streams.items()}}
