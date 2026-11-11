"""Synthetic disarmed mode-diagnostic traces. No ROS connection or commands."""
import subprocess
import sys
from types import SimpleNamespace

import pytest

from dronebench.disarmed_offboard import DisarmedOffboardProbe, ProbeAction, native_commander_ack_matches
from tools.disarmed_offboard_probe import candidate_topics
from sim.gazebo_smoke import disarmed_offboard_passed, observation_seconds, run


class Trace:
    def __init__(self, probe_mode=True):
        self.wall = self.sim = 0.
        self.probe = DisarmedOffboardProbe(started_wall=0., probe_mode=probe_mode)
        self.status = {'armed': False, 'disarmed': True, 'offboard': False, 'auto_loiter': True,
                       'preflight': False, 'failsafe': False, 'system_id': 1, 'component_id': 1}
        self.position = {'x': 2., 'y': 3., 'z': -.2, 'vx': 0., 'vy': 0., 'vz': 0., 'heading': .1,
                         'xy_valid': True, 'z_valid': True, 'v_xy_valid': True, 'v_z_valid': True,
                         'heading_good_for_control': True, 'xy_reset_counter': 0, 'z_reset_counter': 0,
                         'vxy_reset_counter': 0, 'vz_reset_counter': 0, 'heading_reset_counter': 0}
        self.land = {'landed': True, 'freefall': False}
        self.actions = []

    def tick(self, *, sim_dt=.05, freeze=None, duplicate=None, periods=None):
        self.wall = round(self.wall+.05, 8)
        self.sim = round(self.sim+sim_dt, 8)
        for name in ('status', 'position', 'land'):
            old = self.probe.telemetry.streams[name].stamp
            if name == freeze or periods and name in periods and old is not None and self.sim-old < periods[name]:
                continue
            stamp = old if name == duplicate else self.sim
            self.probe.observe(name, stamp_s=stamp, received_wall=self.wall, data=getattr(self, name))
        actions = self.probe.step(now_sim=self.sim, now_wall=self.wall)
        self.actions.extend(actions)
        for action in actions:
            if action.kind == 'stream':
                self.probe.streamed(now_sim=self.sim, now_wall=self.wall)
        return actions

    def advance(self, seconds, **kwargs):
        for _ in range(round(seconds/.05)):
            self.tick(**kwargs)

    def mode_pending(self):
        self.advance(31.8)
        assert self.probe.state == 'WAIT_MODE'
        return self

    def offboard(self):
        self.mode_pending()
        self.status.update(auto_loiter=False, offboard=True)
        self.probe.acknowledge(result='accepted', stamp_s=self.sim+.001, received_wall=self.wall+.001)
        self.tick()
        assert self.probe.state == 'OFFBOARD_OBSERVE'
        return self


def test_default_read_only_never_recommends_control_even_when_startup_window_passes():
    trace = Trace(probe_mode=False)
    trace.advance(40.)
    assert not trace.actions
    assert trace.probe.report()['startup_window_satisfied']
    assert not trace.probe.report()['mode_requested']
    assert not trace.probe.report()['offboard_preflight_window_satisfied']


def test_transient_startup_age_failure_restarts_full_fresh_window_without_output():
    trace = Trace()
    trace.advance(2.)
    assert trace.probe.startup_since is not None and not trace.actions
    trace.tick(sim_dt=.2, freeze='position')  # Strict 0.1sim position freshness fails.
    assert trace.probe.state == 'WAITING' and trace.probe.startup_since is None
    failure = trace.probe.report()['last_health_failure']
    assert failure['now_sim_s'] == trace.sim and failure['now_wall_s'] == trace.wall
    assert failure['stream_ages']['position']['source_age_sim_s'] > .1
    trace.tick()
    restarted = trace.wall
    trace.advance(29.9)
    assert not trace.actions  # The preceding two seconds do not count.
    trace.advance(.3)
    assert trace.probe.state == 'PRESTREAM'
    assert trace.probe.first_stream_wall-restarted >= 30.


def test_startup_source_regression_and_identity_change_still_latch_faults():
    trace = Trace()
    trace.advance(2.)
    trace.probe.observe('position', stamp_s=trace.sim-1., received_wall=trace.wall, data=trace.position)
    trace.tick()
    assert trace.probe.state == 'FAULT' and not trace.actions
    trace = Trace()
    trace.advance(2.)
    trace.status['system_id'] = 2
    trace.tick()
    assert trace.probe.state == 'FAULT' and not trace.actions


def test_auto_loiter_native_preflight_false_is_recorded_not_forged():
    trace = Trace()
    trace.advance(29.9)
    assert not trace.actions
    trace.advance(1.9)
    assert trace.probe.state == 'WAIT_MODE'
    assert trace.probe.status()['preflight'] is False
    assert sum(action.kind == 'mode_offboard' for action in trace.actions) == 1
    assert trace.probe.stream_count >= 22
    assert trace.probe.mode_request[0]-trace.probe.first_stream_sim >= 1.1
    assert set(action.kind for action in trace.actions) == {'stream', 'mode_offboard'}


def test_native_offboard_preflight_must_be_true_continuously_for_thirty_seconds():
    trace = Trace().offboard()
    trace.advance(5.)
    assert trace.probe.preflight_since is None
    trace.status['preflight'] = True
    trace.advance(15.)
    trace.status['preflight'] = False
    trace.tick()
    assert trace.probe.preflight_since is None
    trace.status['preflight'] = True
    trace.advance(29.9)
    assert trace.probe.state == 'OFFBOARD_OBSERVE'
    trace.advance(.2)
    assert trace.probe.state == 'COMPLETE'
    assert trace.probe.report()['offboard_preflight_window_satisfied']
    assert trace.probe.status()['disarmed'] is True
    assert all(action.position == (2., 3., -.2) for action in trace.actions if action.kind == 'stream')
    assert trace.tick() == []


@pytest.mark.parametrize('bad_ack', ['absent', 'stale', 'rejected'])
def test_mode_status_without_matching_fresh_accepted_ack_never_qualifies(bad_ack):
    trace = Trace().mode_pending()
    trace.status.update(auto_loiter=False, offboard=True, preflight=True)
    if bad_ack != 'absent':
        trace.probe.acknowledge(result='rejected' if bad_ack == 'rejected' else 'accepted',
                                stamp_s=trace.sim-2 if bad_ack == 'stale' else trace.sim+.001,
                                received_wall=trace.wall+.001)
    trace.advance(3.2)
    assert trace.probe.state == 'FAULT'
    assert not trace.probe.report()['offboard_preflight_window_satisfied']
    assert sum(action.kind == 'mode_offboard' for action in trace.actions) == 1


def test_fresh_ack_can_precede_lower_rate_actual_mode_report():
    trace = Trace().mode_pending()
    trace.probe.acknowledge(result='accepted', stamp_s=trace.sim+.001, received_wall=trace.wall+.001)
    trace.tick()
    trace.advance(.7)
    trace.status.update(auto_loiter=False, offboard=True)
    trace.tick()
    assert trace.probe.state == 'OFFBOARD_OBSERVE'


@pytest.mark.parametrize('failure', ['armed', 'stale', 'identity', 'reset', 'movement', 'mode'])
def test_any_authority_or_estimator_failure_stops_all_output(failure):
    trace = Trace().offboard()
    if failure == 'armed':
        trace.status.update(armed=True, disarmed=False)
    elif failure == 'identity':
        trace.status['system_id'] = 2
    elif failure == 'reset':
        trace.position['xy_reset_counter'] = 1
    elif failure == 'movement':
        trace.position['z'] += .051
    elif failure == 'mode':
        trace.status.update(offboard=False, auto_loiter=True)
    trace.advance(.7, duplicate='position' if failure == 'stale' else None)
    assert trace.probe.state == 'FAULT'
    assert trace.tick() == []
    assert not trace.probe.report()['offboard_preflight_window_satisfied']
    assert set(action.kind for action in trace.actions) <= {'stream', 'mode_offboard'}


def test_measured_slow_status_and_land_cadence_remains_valid():
    trace = Trace()
    trace.advance(33., sim_dt=.04, periods={'status': .5, 'land': 1.})
    assert trace.probe.state == 'WAIT_MODE'
    assert trace.probe.telemetry.streams['land'].advances > 20


def test_native_preflight_false_reaches_deadline_without_optimistic_success():
    trace = Trace().offboard()
    trace.advance(90.)
    assert trace.probe.state == 'FAULT'
    assert 'deadline' in trace.probe.reason
    assert not trace.probe.report()['offboard_preflight_window_satisfied']


def test_scope_and_cli_do_not_permit_unreviewed_mode_control(tmp_path):
    with pytest.raises(ValueError, match='outside'):
        ProbeAction('unsupported_command')
    output = tmp_path/'must-not-exist.json'
    result = subprocess.run([sys.executable, 'tools/disarmed_offboard_probe.py', '--probe-mode', '--output', str(output)],
                             capture_output=True, text=True, timeout=5)
    assert result.returncode == 2 and not output.exists()
    graph = [('/fmu/in/vehicle_command', ['px4_msgs/msg/VehicleCommand']),
             ('/fmu/in/vehicle_command_mode_executor', ['px4_msgs/msg/VehicleCommand'])]
    assert candidate_topics(graph, direction='in', basename='vehicle_command',
                            type_name='px4_msgs/msg/VehicleCommand') == ['/fmu/in/vehicle_command']


def test_native_commander_ack_uses_false_external_flag_even_for_external_request():
    # Pinned v1.16 Commander.cpp:answer_command initializes ACK{} and never
    # copies cmd.from_external. mavlink_receiver sets true only on received ACKs.
    ack = SimpleNamespace(command=176, target_system=1, target_component=191, from_external=False)
    options = {'expected_commands': (176,), 'target_system': 1, 'target_component': 191, 'request_sent': True}
    assert native_commander_ack_matches(ack, **options)
    ack.from_external = True
    assert not native_commander_ack_matches(ack, **options)
    ack.from_external = False
    for field, wrong in (('command', 999), ('target_system', 2), ('target_component', 1)):
        previous = getattr(ack, field)
        setattr(ack, field, wrong)
        assert not native_commander_ack_matches(ack, **options)
        setattr(ack, field, previous)
    assert not native_commander_ack_matches(ack, **dict(options, request_sent=False))


def test_native_ack_matching_preserves_state_and_freshness_requirements():
    trace = Trace().mode_pending()
    ack = SimpleNamespace(command=176, target_system=1, target_component=191, from_external=False)
    assert native_commander_ack_matches(ack, expected_commands=(176,), target_system=1,
                                        target_component=191, request_sent=True)
    trace.probe.acknowledge(result='accepted', stamp_s=trace.sim+.001, received_wall=trace.wall+.001)
    trace.tick()
    assert trace.probe.state == 'WAIT_MODE'  # Native ACK alone does not assert mode.
    trace.status.update(auto_loiter=False, offboard=True)
    trace.tick()
    assert trace.probe.state == 'OFFBOARD_OBSERVE'
    assert not trace.probe.report()['offboard_preflight_window_satisfied']


@pytest.mark.parametrize('missing', ['bootstrap', 'gcs', 'heading', 'mutual'])
def test_harness_rejects_unqualified_inspection_before_creating_outputs(tmp_path, missing):
    options = {'with_px4': True, 'vio_config': tmp_path/'vio.yaml', 'diagnostic_rig': True,
               'external_vision': True, 'bootstrap_fixture': True, 'ground_station': tmp_path/'qgc',
               'align_heading': True, 'inspect_offboard_disarmed': True}
    if missing == 'bootstrap':
        options['bootstrap_fixture'] = False
    elif missing == 'gcs':
        options['ground_station'] = None
    elif missing == 'heading':
        options['align_heading'] = False
    else:
        options['inspect_hover'] = True
    output = tmp_path/'must-not-create'
    with pytest.raises(ValueError):
        run(tmp_path, tmp_path, tmp_path/'scene', output, **options)
    assert not output.exists()


def test_harness_duration_leaves_room_for_detach_and_whole_diagnostic():
    assert observation_seconds(vio_config=True, inspect_offboard_disarmed=True) == 195
    assert observation_seconds(vio_config=True, inspect_hover=True) == 130
    assert observation_seconds(vio_config=True) == 90
    assert observation_seconds(vio_config=None) == 30


def test_harness_keeps_mode_qualification_separate_from_transport_and_flight():
    report = {'state': 'COMPLETE', 'probe_mode': True, 'control_publishers_created': True,
              'startup_window_satisfied': True, 'startup_window_s': 30.,
              'offboard_preflight_window_satisfied': True, 'offboard_preflight_window_s': 30.,
              'mode_ack_accepted': True, 'mode_commands_sent': 1, 'heartbeat_samples_published': 1000,
              'exception': None, 'telemetry': {'status': {'last_fields': {'offboard': True, 'preflight': True,
                                                'armed': False, 'disarmed': True}},
                                            'land': {'last_fields': {'landed': True}}}}
    assert disarmed_offboard_passed(report, 0)
    assert not disarmed_offboard_passed(report, 1)
    assert not disarmed_offboard_passed({'sensor_streams_observed': True}, 0)
    report['telemetry']['status']['last_fields']['armed'] = True
    assert not disarmed_offboard_passed(report, 0)
    assert 'flight_ready' not in report
