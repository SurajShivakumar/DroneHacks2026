"""Mission authority/phase decisions using real PX4 telemetry, no ROS I/O.

The flight adapter supplies approved navigation actions. This module does not
infer checkpoints, landing-pad clearance, or authorize flight on its own.
"""
from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, sin

from dronebench.flight_guard import Action, HoverGuard


@dataclass(frozen=True)
class MissionPlan:
    takeoff_m: float = .5
    climb_mps: float = .25
    hold_s: float = 2.
    max_takeoff_xy_m: float = .03
    max_heading_error_rad: float = .1
    idle_health_wall_s: float = 30.
    disarmed_preflight_wall_s: float = 30.
    max_wall_s: float = 240.
    max_leg_sim_s: float = 120.
    task: str = 'hover'
    return_enabled: bool = False
    # Nov 7: climb-rate velocity feedforward during the takeoff ramp; without it
    # PX4 creeps the last ~30 cm for 6-7 s (v84 trace). Envelope checks unchanged.
    takeoff_feedforward: bool = False

    def __post_init__(self):
        if self.task not in ('hover', 'trajectory', 'ppo'):
            raise ValueError('unknown mission task')
        if any(not isfinite(v) or v <= 0 for v in (
                self.takeoff_m, self.climb_mps, self.hold_s, self.max_takeoff_xy_m,
                self.max_heading_error_rad, self.idle_health_wall_s,
                self.disarmed_preflight_wall_s, self.max_wall_s, self.max_leg_sim_s)):
            raise ValueError('positive finite mission limits required')
        if self.idle_health_wall_s < 30 or self.disarmed_preflight_wall_s < 30:
            raise ValueError('continuous thirty-second health windows required')
        # Nov 6 (team request, speed series): climb cap raised 0.25 -> 0.6 m/s.
        # The 3 cm XY / heading takeoff envelope and 2 cm settle band are unchanged.
        if self.takeoff_m > 1.2 or self.climb_mps > .6:
            raise ValueError('takeoff exceeds reviewed initial envelope')
        if type(self.takeoff_feedforward) is not bool:
            raise ValueError('takeoff_feedforward must be boolean')
        if type(self.return_enabled) is not bool or self.task == 'hover' and self.return_enabled:
            raise ValueError('return requires a navigation task')


class CourseMission(HoverGuard):
    """Reuse the reviewed telemetry freshness and command ACK contracts.

    After an approved execution: idle health30s -> disarmed Offboard prestream
    -> ACK and actual mode -> native Offboard preflight30s -> arm ACK/state ->
    bounded takeoff -> navigation -> Land ACK/mode -> landed AND disarmed.
    A return is a new preflight/takeoff sequence, gated on independent outbound
    evaluation. It never re-arms just because a policy says 'finished'.
    """
    def __init__(self, *, started_wall, plan=MissionPlan(), execute=False, approval_reference=None):
        super().__init__(started_wall=started_wall, execute=execute, approval_reference=approval_reference)
        self.plan = plan
        self.state = 'WAITING'
        self.native_since = self.takeoff_started = self.leg_started = self.navigation_started = None
        self.leg = 'outbound'
        self.outbound_verified = False
        self.nav_ready_to_land = False
        self.nav_evidence = None
        self.outbound_complete = False
        self.hold_since = None

    def health(self, *, now_sim, now_wall):
        error = super().health(now_sim=now_sim, now_wall=now_wall)
        if error:
            return error
        if self._status().get('gcs_connection_lost') is not False:
            return 'actual ground station connection missing or lost'
        return None

    def navigation_finished(self, *, evidence, outbound_verified=False):
        """Called only after independent fresh target/checkpoint evaluation.

        The adapter must check the evidence; a string alone is not a grader.
        """
        if self.state != 'NAVIGATE' or not isinstance(evidence, str) or not evidence.strip():
            raise ValueError('navigation phase and independent evidence required')
        self.nav_ready_to_land, self.nav_evidence = True, evidence
        if self.leg == 'outbound':
            self.outbound_verified = outbound_verified is True

    def abort_navigation(self, *, reason, now_sim, now_wall):
        """Native Land for a planning/tracking failure with fresh owned authority.

        Telemetry or authority failures still stop publication immediately.
        Landing an aborted navigation never completes or qualifies the course.
        """
        if (not reason or self.state!='NAVIGATE' or not self.execute or not self.owns
                or self.health(now_sim=now_sim,now_wall=now_wall) is not None
                or self._status().get('armed') is not True or self._status().get('offboard') is not True):
            return self._fault('navigation abort lacks fresh owned authority')
        self.abort_reason=self.reason=str(reason)
        self.events.append({'kind':'navigation_abort','reason':str(reason),'sim_s':now_sim,'wall_s':now_wall})
        return self._land(now_sim,now_wall)

    def abort_for_vio_quality(self, *, reason, now_sim, now_wall):
        """Land on a sustained VIO tracking-quality loss with owned authority."""
        if (not reason or self.state not in ('TAKEOFF', 'NAVIGATE') or not self.execute
                or not self.owns or self.health(now_sim=now_sim, now_wall=now_wall) is not None
                or self._status().get('armed') is not True or self._status().get('offboard') is not True):
            return self._fault('VIO quality abort lacks fresh owned authority')
        self.abort_reason = self.reason = str(reason)
        self.events.append({'kind': 'vio_quality_abort', 'reason': str(reason),
                            'sim_s': now_sim, 'wall_s': now_wall})
        return self._land(now_sim, now_wall)

    def _fault(self, reason):
        self.state, self.reason, self.owns = 'FAULT', reason, False
        return []  # Stopping a publisher is not a claim of landing.

    def _stream(self, target=None):
        return [Action('stream', self.origin if target is None else target, self.yaw)]

    def _land(self, now_sim, now_wall):
        self.state, self.land_requested, self.owns = 'LANDING', True, False
        return [self._request('land', now_sim, now_wall)]

    def step(self, *, now_sim, now_wall):
        if self.state in self.terminal:
            return []
        if not all(isfinite(v) for v in (now_sim, now_wall)) or now_sim < 0:
            return self._fault('invalid current clock')
        if self.last_tick_sim is not None and (now_sim < self.last_tick_sim or now_wall < self.last_tick_wall):
            return self._fault('current clock regressed')
        if self.state != 'WAITING' and self.last_tick_wall is not None:
            if now_wall-self.last_tick_wall > .25 or now_sim-self.last_tick_sim > .25:
                return self._fault('control heartbeat cadence lost')
        self.last_tick_sim, self.last_tick_wall = now_sim, now_wall
        if now_wall-self.started_wall >= self.plan.max_wall_s:
            return self._fault('mission wall deadline')
        error = self.health(now_sim=now_sim, now_wall=now_wall)
        status, position = self._status(), self.streams['position'].data or {}
        land = self.streams['land'].data or {}
        if self.state == 'WAITING':
            if any(s.invalid for s in self.streams.values()):
                return self._fault(error or 'invalid telemetry continuity')
            ready = (error is None and status.get('disarmed') is True and land.get('landed') is True
                     and (status.get('auto_loiter') is True or self.leg == 'return' and status.get('land_mode') is True))
            if not ready:
                self.ready_since, self.reason = None, error or 'require original idle mode, landed and disarmed'
                return []
            if self.ready_since is None:
                self.ready_since = now_wall
            self.reason = 'observing idle health; native Offboard preflight still required'
            if now_wall-self.ready_since < self.plan.idle_health_wall_s or not self.execute:
                return []
            self.origin = tuple(position[k] for k in ('x', 'y', 'z'))
            self.origin_resets = tuple(position.get(k) for k in self.reset_fields)
            if any(v is None for v in self.origin_resets):
                return self._fault('missing estimator reset counters')
            self.target_ids = status['system_id'], status['component_id']
            self.yaw = position['heading']
            self.state, self.reason = 'PRESTREAM', None
            self.events.append({'kind': 'idle_health30s', 'sim_s': now_sim, 'wall_s': now_wall, 'leg': self.leg})
        elif error:
            return self._fault(error)
        if tuple(position.get(k) for k in self.reset_fields) != self.origin_resets:
            return self._fault('estimator reference/reset changed')
        if self.leg_started is not None and now_sim-self.leg_started > self.plan.max_leg_sim_s:
            return self._fault('leg simulation deadline')
        if self.state == 'LANDING':
            self.land_acknowledged |= self._ack('land', now_sim, now_wall) == 'accepted'
            self.land_mode_observed |= status.get('land_mode') is True
            if self._ack('land', now_sim, now_wall) == 'rejected':
                return self._fault('Land rejected; control handed off')
            if not status.get('offboard') and not status.get('land_mode'):
                return self._fault('authority changed after Land request')
            if self.land_acknowledged and self.land_mode_observed and land.get('landed') and status.get('disarmed'):
                self.events.append({'kind': 'landed_disarmed', 'leg': self.leg, 'sim_s': now_sim, 'wall_s': now_wall,
                                    'landed_source_sim_s':self.streams['land'].stamp,
                                    'disarmed_source_sim_s':self.streams['status'].stamp,
                                    'navigation_evidence': self.nav_evidence})
                if self.abort_reason is not None:
                    self.state='ABORT_LANDED'
                    return []
                if self.leg == 'outbound':
                    self.outbound_complete = True
                # A later explicit adapter call starts the qualified return;
                # do not automatically publish/arm on this terminal observation.
                self.state = 'LANDED' if self.plan.return_enabled and self.leg == 'outbound' else 'COMPLETE'
            return []
        if self.state == 'LANDED':
            return []
        if self.state in ('PRESTREAM', 'WAIT_MODE', 'DISARMED_CHECK', 'WAIT_ARM'):
            if status.get('armed') and self.state != 'WAIT_ARM':
                return self._fault('unexpected armed authority')
            if not status.get('armed') and (not status.get('disarmed') or not land.get('landed')):
                return self._fault('pre-arm landed/disarmed state lost')
            if hypot(position['x']-self.origin[0], position['y']-self.origin[1]) > self.plan.max_takeoff_xy_m:
                return self._fault('ground hold position envelope exceeded')
        elif not (self.owns and status.get('armed') and status.get('offboard')):
            return self._fault('armed Offboard authority lost')
        if self.state == 'PRESTREAM':
            if not (status.get('auto_loiter') or self.leg == 'return' and status.get('land_mode')):
                return self._fault('original idle mode lost before mode request')
            if self.stream_count >= 22 and now_sim-self.first_stream_sim > 1 and now_wall-self.first_stream_wall > 1:
                self.state = 'WAIT_MODE'
                return self._stream()+[self._request('mode_offboard', now_sim, now_wall)]
        elif self.state == 'WAIT_MODE':
            if not (status.get('auto_loiter') or status.get('offboard') or self.leg=='return' and status.get('land_mode')):
                return self._fault('unexpected authority during disarmed mode request')
            ack = self._ack('mode_offboard', now_sim, now_wall)
            if ack == 'rejected' or self._timed_out('mode_offboard', now_sim, now_wall):
                return self._fault('Offboard mode rejected or timed out')
            if ack == 'accepted' and status.get('offboard'):
                self.state = 'DISARMED_CHECK'
        if self.state == 'DISARMED_CHECK':
            if not status.get('offboard'):
                return self._fault('Offboard authority lost during native preflight')
            if status.get('preflight') is not True:
                self.native_since = None
            elif self.native_since is None:
                self.native_since = now_wall
            elif now_wall-self.native_since >= self.plan.disarmed_preflight_wall_s:
                self.state = 'WAIT_ARM'
                self.events.append({'kind': 'native_offboard_preflight30s', 'sim_s': now_sim, 'wall_s': now_wall, 'leg': self.leg})
                return self._stream()+[self._request('arm', now_sim, now_wall)]
        elif self.state == 'WAIT_ARM':
            if not status.get('offboard'):
                return self._fault('mode lost before arm acknowledgement')
            ack = self._ack('arm', now_sim, now_wall)
            if ack == 'rejected' or self._timed_out('arm', now_sim, now_wall):
                return self._fault('arm rejected or timed out')
            if not status.get('armed') and status.get('preflight') is not True:
                return self._fault('native pre-arm check lost')
            if ack == 'accepted' and status.get('armed'):
                self.owns, self.state = True, 'TAKEOFF'
                self.takeoff_started = self.leg_started = now_sim
                self.events.append({'kind':'armed_observed','leg':self.leg,'sim_s':now_sim,'wall_s':now_wall,
                                    'armed_status_source_sim_s':self.streams['status'].stamp})
        if self.state == 'TAKEOFF':
            height = self.origin[2]-position['z']
            if (hypot(position['x']-self.origin[0], position['y']-self.origin[1]) > self.plan.max_takeoff_xy_m
                    or not -.1 <= height <= self.plan.takeoff_m+.15
                    or abs(atan2(sin(position['heading']-self.yaw), cos(position['heading']-self.yaw))) > self.plan.max_heading_error_rad):
                return self._fault('takeoff position/heading envelope exceeded')
            rise = min(self.plan.takeoff_m, self.plan.climb_mps*(now_sim-self.takeoff_started))
            target = (*self.origin[:2], self.origin[2]-rise)
            # The narrow-course reference requires a tighter handoff than the
            # standalone hover's altitude acceptance band.
            settle_band=.06 if self.plan.task=='hover' else .02
            settled = (rise >= self.plan.takeoff_m and abs(height-self.plan.takeoff_m) < settle_band
                       and hypot(position['vx'], position['vy']) < .1 and abs(position['vz']) < .1)
            if not settled:
                self.hold_since = None
            elif self.hold_since is None:
                self.hold_since = now_sim
            elif now_sim-self.hold_since >= self.plan.hold_s:
                if self.plan.task == 'hover':
                    return self._land(now_sim, now_wall)
                self.state, self.navigation_started = 'NAVIGATE', now_sim
                self.events.append({'kind':'navigation_started','leg':self.leg,'sim_s':now_sim,'wall_s':now_wall})
            if self.plan.takeoff_feedforward:
                climbing = rise < self.plan.takeoff_m
                return [Action('stream', target, self.yaw, (0., 0., -self.plan.climb_mps if climbing else 0.))]
            return self._stream(target)
        if self.state == 'NAVIGATE':
            if self.nav_ready_to_land:
                return self._land(now_sim, now_wall)
            return [Action('navigate')]  # Adapter must supply a checked task output.
        return self._stream()

    def start_return(self, *, evidence, now_sim, now_wall):
        if (self.state != 'LANDED' or not self.plan.return_enabled or not self.outbound_verified
                or not evidence or self.health(now_sim=now_sim, now_wall=now_wall) is not None
                or self._status().get('disarmed') is not True or self.streams['land'].data.get('landed') is not True):
            raise ValueError('qualified outbound landing/disarm and fresh return-route evidence required')
        # After confirmed landing, AUTO_LAND is a supported disarmed starting
        # mode for the return's new prestream/mode/preflight sequence.
        self.state, self.leg, self.ready_since = 'WAITING', 'return', None
        self.origin = self.origin_resets = self.target_ids = None
        self.native_since = self.takeoff_started = self.leg_started = self.navigation_started = None
        self.first_stream_sim = self.first_stream_wall = None
        self.stream_count = 0
        self.pending.clear(); self.acks.clear(); self.validated_acks.clear()
        self.nav_ready_to_land = self.land_requested = self.land_acknowledged = self.land_mode_observed = False
        self.events.append({'kind': 'return_preflight_requested', 'evidence': evidence, 'sim_s': now_sim, 'wall_s': now_wall})

    def report(self, **kwargs):
        result = super().report(**kwargs)
        result.update(kind='course_mission_supervisor_not_a_grader', leg=self.leg,
                      abort_reason=self.abort_reason,
                      outbound_complete=self.outbound_complete, outbound_verified=self.outbound_verified,
                      navigation_evidence=self.nav_evidence, plan=vars(self.plan),
                      flight_ready_claimed=False)
        now_wall=self.last_tick_wall if kwargs.get('now_wall') is None else kwargs['now_wall']
        result['idle_health_window_s']=max(0.,now_wall-self.ready_since) if now_wall is not None and self.ready_since is not None else 0.
        result['native_offboard_preflight_observed']=any(event['kind']=='native_offboard_preflight30s' for event in self.events)
        return result
