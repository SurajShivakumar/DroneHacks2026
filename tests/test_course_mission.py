"""Authority/phase traces; no ROS, simulator, or aircraft commands."""
import pytest
from dronebench.course_mission import CourseMission, MissionPlan


class Trace:
    def __init__(self, *, execute=True, task='hover', return_enabled=False):
        self.time = 0.
        self.guard = CourseMission(started_wall=0., execute=execute,
            approval_reference='test fixture, NOT flight approval' if execute else None,
            plan=MissionPlan(task=task, return_enabled=return_enabled))
        self.status = dict(armed=False, disarmed=True, offboard=False, auto_loiter=True,
            startup_mode_allowed=True, land_mode=False, preflight=False, failsafe=False,
            system_id=1, component_id=1, gcs_connection_lost=False)
        self.position = dict(x=0., y=0., z=0., vx=0., vy=0., vz=0., heading=0.,
            xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True, heading_good_for_control=True,
            **{k: 0 for k in self.guard.reset_fields})
        self.land = dict(landed=True, freefall=False)
        self.actions = []

    def tick(self):
        self.time = round(self.time+.05, 8)
        for name in ('status', 'position', 'land'):
            self.guard.observe(name, stamp_s=self.time, received_wall=self.time, data=getattr(self, name))
        actions = self.guard.step(now_sim=self.time, now_wall=self.time)
        for action in actions:
            if action.kind == 'stream':
                self.guard.streamed(now_sim=self.time, now_wall=self.time)
        self.actions += actions
        return actions

    def advance(self, seconds):
        for _ in range(round(seconds/.05)):
            self.tick()

    def ack(self, name, result='accepted'):
        self.guard.acknowledge(name, result=result, stamp_s=self.time+.001, received_wall=self.time+.001)

    def offboard(self):
        self.advance(31.5)
        assert self.guard.state == 'WAIT_MODE'
        self.status.update(offboard=True, auto_loiter=False)
        self.ack('mode_offboard'); self.tick()
        assert self.guard.state == 'DISARMED_CHECK'
        return self

    def airborne(self):
        self.offboard()
        self.status['preflight'] = True
        self.advance(30.1)
        assert self.guard.state == 'WAIT_ARM'
        self.status.update(armed=True, disarmed=False)
        self.ack('arm'); self.tick()
        self.land['landed'] = False
        return self


def test_default_readonly_and_native_checks_cannot_be_skipped():
    trace = Trace(execute=False)
    trace.advance(80)
    assert not trace.actions
    trace = Trace().offboard()
    trace.advance(35)
    assert not any(a.kind == 'arm' for a in trace.actions)
    trace.status['preflight'] = True
    trace.advance(29)
    assert not any(a.kind == 'arm' for a in trace.actions)
    trace.status['preflight'] = False; trace.tick()
    trace.status['preflight'] = True; trace.advance(29)
    assert not any(a.kind == 'arm' for a in trace.actions)
    trace.advance(1.1)
    assert sum(a.kind == 'arm' for a in trace.actions) == 1


def test_actual_mode_without_matching_ack_never_arms():
    trace = Trace(); trace.advance(31.5)
    trace.status.update(offboard=True, auto_loiter=False, preflight=True)
    trace.advance(3.2)
    assert trace.guard.state == 'FAULT'
    assert not any(a.kind == 'arm' for a in trace.actions)


def test_takeoff_and_real_landing_require_ack_mode_landed_and_disarmed():
    trace = Trace().airborne()
    trace.advance(1)
    assert trace.actions[-1].position[2] == pytest.approx(-.25)
    trace.position['z'] = -.5
    trace.advance(3.2)
    assert trace.guard.state == 'LANDING'
    count = len(trace.actions)
    trace.status.update(offboard=False, land_mode=True)
    trace.ack('land'); trace.tick()
    trace.land['landed'] = True; trace.tick()
    assert trace.guard.state == 'LANDING'
    trace.status.update(armed=False, disarmed=True); trace.tick()
    assert trace.guard.state == 'COMPLETE'
    assert len(trace.actions) == count


def test_takeover_and_takeoff_envelope_failure_stop_output():
    trace = Trace().airborne()
    trace.status['offboard'] = False
    assert trace.tick() == [] and trace.guard.state == 'FAULT'
    trace = Trace().airborne(); trace.position['x'] = .04
    assert trace.tick() == [] and trace.guard.state == 'FAULT'


def test_return_needs_confirmed_outbound_and_separate_evidence():
    trace = Trace(task='trajectory', return_enabled=True).airborne()
    trace.position['z'] = -.5; trace.advance(4.2)
    assert trace.guard.state == 'NAVIGATE'
    trace.guard.navigation_finished(evidence='fixture checkpoint proof', outbound_verified=False)
    trace.tick()
    trace.status.update(offboard=False, land_mode=True); trace.ack('land'); trace.tick()
    trace.land['landed'] = True; trace.status.update(armed=False, disarmed=True); trace.tick()
    assert trace.guard.state == 'LANDED'
    with pytest.raises(ValueError, match='qualified outbound'):
        trace.guard.start_return(evidence='fixture', now_sim=trace.time, now_wall=trace.time)
    trace.guard.outbound_verified = True
    trace.guard.start_return(evidence='separate fixture return clearance', now_sim=trace.time, now_wall=trace.time)
    trace.tick()
    assert trace.guard.leg == 'return' and trace.guard.state == 'WAITING'
    assert trace.guard.stream_count == 0 and trace.guard.pending == {}


def test_fresh_tracking_abort_lands_without_claiming_course_success():
    trace=Trace(task='trajectory').airborne()
    trace.position['z']=-.5; trace.advance(4.2)
    actions=trace.guard.abort_navigation(reason='tracking exceeded screened margin',
        now_sim=trace.time,now_wall=trace.time)
    assert [action.kind for action in actions]==['land']
    trace.status.update(offboard=False,land_mode=True);trace.ack('land');trace.tick()
    trace.land['landed']=True;trace.status.update(armed=False,disarmed=True);trace.tick()
    assert trace.guard.state=='ABORT_LANDED' and not trace.guard.outbound_complete
    assert trace.guard.report()['abort_reason']=='tracking exceeded screened margin'


def test_tracking_abort_never_publishes_with_lost_authority():
    trace=Trace(task='trajectory').airborne()
    trace.position['z']=-.5;trace.advance(4.2)
    trace.status['offboard']=False;trace.tick()
    assert trace.guard.abort_navigation(reason='tracking',now_sim=trace.time,now_wall=trace.time)==[]
    assert trace.guard.state=='FAULT' and not trace.guard.land_requested
