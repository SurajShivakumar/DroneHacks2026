"""Synthetic telemetry/action traces; no ROS connection, flight or arming."""
from math import isclose
import subprocess
import sys

import pytest

from dronebench.flight_guard import HoverGuard
from tools.hover_qualification import candidate_topics


class Trace:
    def __init__(self, execute=True):
        self.wall = self.sim = 0.
        self.guard = HoverGuard(started_wall=0., execute=execute,
                                approval_reference='synthetic unit test, NOT flight approval' if execute else None)
        self.status = {'armed': False, 'disarmed': True, 'offboard': False, 'land_mode': False,
                       'startup_mode_allowed': True, 'preflight': True, 'failsafe': False,
                       'system_id': 1, 'component_id': 1}
        self.position = {'x': 4., 'y': -2., 'z': 1.2, 'vx': 0., 'vy': 0., 'vz': 0., 'heading': .3,
                         'xy_valid': True, 'z_valid': True, 'v_xy_valid': True, 'v_z_valid': True,
                         'heading_good_for_control': True, 'xy_reset_counter': 0,
                         'z_reset_counter': 0, 'vxy_reset_counter': 0, 'vz_reset_counter': 0, 'heading_reset_counter': 0}
        self.land = {'landed': True, 'freefall': False}
        self.actions = []

    def tick(self, dt=.05, sim_dt=None, freeze=None, duplicate=None, periods=None):
        self.wall = round(self.wall + dt, 8)
        self.sim = round(self.sim + (dt if sim_dt is None else sim_dt), 8)
        for name in ('status', 'position', 'land'):
            old = self.guard.streams[name].stamp
            if periods and name in periods and old is not None and self.sim - old < periods[name]:
                continue
            if name != freeze:
                stamp = self.guard.streams[name].stamp if name == duplicate else self.sim
                self.guard.observe(name, stamp_s=stamp, received_wall=self.wall, data=getattr(self, name))
        actions = self.guard.step(now_sim=self.sim, now_wall=self.wall)
        for action in actions:
            self.actions.append((self.sim, action))
            if action.kind == 'stream':
                self.guard.streamed(now_sim=self.sim, now_wall=self.wall)
        return actions

    def advance(self, seconds, **kwargs):
        for _ in range(round(seconds/.05)):
            self.tick(**kwargs)

    def acknowledge(self, kind, result='accepted', stale=False):
        self.guard.acknowledge(kind, result=result,
                               stamp_s=self.sim-1 if stale else self.sim+.001,
                               received_wall=self.wall+.001)

    def wait_for_mode_request(self):
        self.advance(31.5)
        assert self.guard.state == 'WAIT_MODE'

    def owned(self):
        self.wait_for_mode_request()
        self.status.update(offboard=True, startup_mode_allowed=False)
        self.acknowledge('mode_offboard')
        self.tick()
        assert self.guard.state == 'WAIT_ARM'
        self.status.update(armed=True, disarmed=False)
        self.acknowledge('arm')
        self.tick()
        assert self.guard.state == 'CLIMB' and self.guard.owns
        self.land['landed'] = False
        return self


def test_read_only_default_never_recommends_a_control_message():
    trace = Trace(execute=False)
    trace.advance(40.)
    assert not trace.actions
    assert trace.guard.stream_count == 0
    assert trace.guard.state == 'WAITING'
    assert trace.guard.report()['preflight_window_satisfied'] is True
    assert trace.guard.report()['ready_window_s'] >= 30.
    with pytest.raises(ValueError, match='approval'):
        HoverGuard(started_wall=0., execute=True)


def test_report_only_readiness_does_not_survive_new_invalid_or_stale_telemetry():
    trace = Trace(execute=False)
    trace.advance(35.)
    assert trace.guard.report()['preflight_window_satisfied']
    assert not trace.guard.report(now_sim=trace.sim+.2, now_wall=trace.wall+.2)['preflight_window_satisfied']
    trace.status['preflight'] = False
    trace.tick()
    report = trace.guard.report()
    assert not report['preflight_window_satisfied'] and report['ready_window_s'] == 0.
    assert not trace.actions


def test_discovery_never_confuses_command_with_mode_executor_input():
    graph = [('/fmu/in/vehicle_command', ['px4_msgs/msg/VehicleCommand']),
             ('/fmu/in/vehicle_command_mode_executor', ['px4_msgs/msg/VehicleCommand']),
             ('/fmu/out/vehicle_status_v1', ['px4_msgs/msg/VehicleStatus'])]
    assert candidate_topics(graph, direction='in', basename='vehicle_command',
                            type_name='px4_msgs/msg/VehicleCommand') == ['/fmu/in/vehicle_command']
    assert candidate_topics(graph, direction='out', basename='vehicle_status',
                            type_name='px4_msgs/msg/VehicleStatus') == ['/fmu/out/vehicle_status_v1']


def test_execute_without_recorded_approval_fails_before_ros_or_evidence_creation(tmp_path):
    output = tmp_path / 'must-not-exist.json'
    result = subprocess.run([sys.executable, 'tools/hover_qualification.py', '--execute', '--output', str(output)],
                             capture_output=True, text=True, timeout=5)
    assert result.returncode == 2
    assert 'requires recorded approval-reference' in result.stderr
    assert not output.exists()


def test_thirty_second_fresh_preflight_precedes_any_setpoint():
    trace = Trace()
    trace.advance(29.9)
    assert not trace.actions
    trace.advance(.4)
    assert trace.actions and trace.actions[0][0] >= 30.
    assert all(action.kind == 'stream' for _, action in trace.actions)


def test_measured_lower_rate_status_and_land_are_fresh_without_weakening_position():
    trace = Trace()
    trace.advance(33., sim_dt=.04, periods={'status': .5, 'land': 1.})
    assert trace.guard.state == 'WAIT_MODE'
    assert trace.guard.streams['status'].advances > 40
    assert trace.guard.streams['land'].advances > 20
    assert trace.guard.freshness['position']['source_sim_s'] == .1
    assert trace.guard.health(now_sim=trace.sim, now_wall=trace.wall) is None


def test_duplicate_slow_land_samples_cannot_keep_source_fresh_forever():
    trace = Trace(execute=False)
    trace.advance(2.)
    trace.advance(3.1, sim_dt=.005, duplicate='land')
    assert 'land: stale' in trace.guard.health(now_sim=trace.sim, now_wall=trace.wall)
    assert not trace.actions


def test_mode_request_needs_actual_confirmed_prestream_over_one_second():
    trace = Trace()
    trace.wait_for_mode_request()
    commands = [(stamp, action.kind) for stamp, action in trace.actions if action.kind != 'stream']
    assert commands == [(commands[0][0], 'mode_offboard')]
    assert commands[0][0] - trace.guard.first_stream_sim > 1.
    assert trace.guard.stream_count >= 22
    assert not any(action.kind == 'arm' for _, action in trace.actions)


@pytest.mark.parametrize('failure', ['invalid_xy', 'invalid_heading', 'unknown_armed', 'preflight', 'nonfinite'])
def test_unqualified_estimator_or_status_never_gets_control_publishers(failure):
    trace = Trace()
    if failure == 'invalid_xy':
        trace.position['xy_valid'] = False
    elif failure == 'invalid_heading':
        trace.position['heading_good_for_control'] = False
    elif failure == 'unknown_armed':
        trace.status.update(armed=False, disarmed=False)
    elif failure == 'preflight':
        trace.status['preflight'] = False
    else:
        trace.position['x'] = float('nan')
    trace.advance(40.)
    assert not trace.actions
    assert trace.guard.state == 'WAITING'


@pytest.mark.parametrize('ack', ['absent', 'stale', 'rejected'])
def test_mode_state_alone_or_bad_ack_does_not_allow_arming(ack):
    trace = Trace()
    trace.wait_for_mode_request()
    trace.status.update(offboard=True)
    if ack != 'absent':
        trace.acknowledge('mode_offboard', result='rejected' if ack == 'rejected' else 'accepted', stale=ack == 'stale')
    trace.advance(3.2)
    assert trace.guard.state == 'FAULT'
    assert not any(action.kind == 'arm' for _, action in trace.actions)


def test_fresh_ack_is_retained_while_waiting_for_lower_rate_mode_status():
    trace = Trace()
    trace.wait_for_mode_request()
    trace.acknowledge('mode_offboard')
    trace.tick()  # The actual ACK was fresh here; mode status is not yet updated.
    trace.advance(.7)
    trace.status['offboard'] = True
    trace.tick()
    assert trace.guard.state == 'WAIT_ARM'
    assert sum(action.kind == 'arm' for _, action in trace.actions) == 1


def test_ramp_is_relative_rate_limited_and_hold_needs_actual_height():
    trace = Trace().owned()
    trace.advance(1.)
    command = trace.actions[-1][1]
    assert command.position[:2] == (4., -2.)
    assert isclose(command.position[2], 1.2-.25, abs_tol=1e-8)
    trace.advance(2.)  # Reference has reached target, actual aircraft has not.
    assert trace.guard.state == 'CLIMB'
    assert not trace.guard.land_requested
    trace.position['z'] = .7
    trace.advance(1.9)
    assert trace.guard.state == 'HOLD' and not trace.guard.land_requested
    trace.advance(.2)
    assert trace.guard.state == 'LANDING'
    assert trace.actions[-1][1].kind == 'land'


def test_rejected_arm_is_never_retried_or_forced():
    trace = Trace()
    trace.wait_for_mode_request()
    trace.status.update(offboard=True)
    trace.acknowledge('mode_offboard')
    trace.tick()
    trace.acknowledge('arm', result='rejected')
    trace.advance(.3)
    assert trace.guard.state == 'FAULT'
    assert sum(action.kind == 'arm' for _, action in trace.actions) == 1
    assert not trace.guard.land_requested


def test_nominal_land_handoff_requires_ack_mode_landed_and_disarmed():
    trace = Trace().owned()
    trace.position['z'] = .7
    trace.advance(4.2)
    assert trace.guard.state == 'LANDING'
    action_count = len(trace.actions)
    trace.status.update(offboard=False, land_mode=True)
    trace.acknowledge('land')
    trace.advance(.2)
    trace.land['landed'] = True
    trace.tick()
    assert trace.guard.state == 'LANDING'  # Still armed.
    trace.status.update(armed=False, disarmed=True)
    trace.tick()
    assert trace.guard.state == 'COMPLETE'
    assert len(trace.actions) == action_count  # Never stream after handoff.


@pytest.mark.parametrize('kind', ['no_arrivals', 'frozen_source'])
def test_stale_telemetry_stops_all_setpoints_and_commands(kind):
    trace = Trace().owned()
    for _ in range(12):
        trace.tick(freeze='position' if kind == 'no_arrivals' else None,
                   duplicate='position' if kind == 'frozen_source' else None)
    assert trace.guard.state == 'FAULT' and not trace.guard.owns
    assert not trace.guard.land_requested  # Stale state is not authority to command Land.
    assert trace.tick() == []


def test_pilot_mode_takeover_never_gets_overridden_by_land_or_setpoints():
    trace = Trace().owned()
    trace.status['offboard'] = False
    assert trace.tick() == []
    assert trace.guard.state == 'FAULT'
    assert not trace.guard.land_requested


def test_changed_px4_identity_never_receives_a_land_command():
    trace = Trace().owned()
    trace.status['system_id'] = 2
    assert trace.tick() == []
    assert trace.guard.state == 'FAULT'
    assert not trace.guard.land_requested


@pytest.mark.parametrize('bound', ['xy', 'height', 'reset'])
def test_fresh_owned_envelope_failure_requests_land_once_then_hands_off(bound):
    trace = Trace().owned()
    if bound == 'xy':
        trace.position['x'] += .21
    elif bound == 'height':
        trace.position['z'] -= .66
    else:
        trace.position['xy_reset_counter'] += 1
    assert [action.kind for action in trace.tick()] == ['land']
    assert trace.guard.state == 'LANDING' and not trace.guard.owns
    trace.advance(.2)
    assert sum(action.kind == 'land' for _, action in trace.actions) == 1
    assert trace.tick() == []


def test_source_clock_regression_latches_failure_without_a_new_position_fix():
    trace = Trace().owned()
    trace.guard.observe('position', stamp_s=trace.sim-1., received_wall=trace.wall,
                        data=trace.position)
    assert trace.tick() == []
    assert trace.guard.state == 'FAULT'
    assert 'regressed' in trace.guard.reason


def test_simulation_deadline_requests_land_once_and_never_claims_completion():
    trace = Trace().owned()
    trace.advance(29.)  # Remain below target so no normal hold/landing completion.
    assert trace.guard.state == 'FAULT'
    assert trace.guard.land_requested
    assert sum(action.kind == 'land' for _, action in trace.actions) == 1
    assert trace.guard.state != 'COMPLETE'


def test_fast_simulation_cannot_arm_with_insufficient_sim_time_heartbeat_rate():
    trace = Trace()
    trace.advance(30.3)
    assert trace.guard.state == 'PRESTREAM'
    trace.tick(sim_dt=.3)
    assert trace.guard.state == 'FAULT'
    assert not any(action.kind == 'arm' for _, action in trace.actions)
