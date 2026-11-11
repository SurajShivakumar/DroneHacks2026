import json
from pathlib import Path
import subprocess
import sys
import pytest

from tools.course_flight import validate_execution_contract, validate_bounded_qualification
from dronebench.course_mission import MissionPlan
from sim.gazebo_smoke import mission_result_passed


def test_cad_visuals_do_not_unlock_flight_with_unknown_actuators():
    with pytest.raises(ValueError, match='incomplete'):
        validate_execution_contract({'flight_model_ready': False, 'motor_plugins_enabled': False,
                                     'missing': ['propeller diameter']})


def test_partial_route_qualification_is_bounded_and_cannot_request_return():
    settings={'bounded_navigation_qualification':True}
    validate_bounded_qualification(settings,MissionPlan(task='trajectory',max_leg_sim_s=90),40)
    with pytest.raises(ValueError,match='no return'):
        validate_bounded_qualification(settings,MissionPlan(task='trajectory',return_enabled=True,max_leg_sim_s=90),40)
    with pytest.raises(ValueError,match='trajectory'):
        validate_bounded_qualification(settings,MissionPlan(task='trajectory',max_leg_sim_s=90),61)


def test_execute_refuses_incomplete_airframe_before_ros_or_any_output(tmp_path):
    contract=tmp_path/'airframe.json'
    contract.write_text(json.dumps({'flight_model_ready':False,'missing':['propeller diameter']}))
    output=tmp_path/'never-created.json'
    result=subprocess.run([sys.executable,'tools/course_flight.py','--settings','configs/sky_beta_hover.json',
        '--airframe-contract',str(contract),'--output',str(output),'--execute','--shared-simulation-clock',
        '--approval-reference','unit test, NOT actual team approval'],capture_output=True,text=True)
    assert result.returncode == 2 and 'incomplete' in result.stderr
    assert not output.exists()


def test_transport_pass_does_not_hide_readonly_mission_failure():
    assert not mission_result_passed({'sensor_streams_observed':True,'state':'FAULT'},0,False)
    report={'exception':None,'state':'WAITING','control_publishers_created':False,'idle_health_window_s':31.,
        'streams':{'status':{'last_fields':{'disarmed':True,'armed':False}},'land':{'last_fields':{'landed':True}}}}
    assert mission_result_passed(report,0,False)
    assert not mission_result_passed(report,1,False)
    assert not mission_result_passed(report,0,True)
    report['exception']='float32 serialization failed'
    assert not mission_result_passed(report,0,False)


def test_takeoff_climb_cap_is_point_six_mps():
    import pytest
    from dronebench.course_mission import MissionPlan
    MissionPlan(task='trajectory', climb_mps=.6, hold_s=.5)
    with pytest.raises(ValueError, match='initial envelope'):
        MissionPlan(task='trajectory', climb_mps=.61)


def test_takeoff_feedforward_flag_validates():
    import pytest
    from dronebench.course_mission import MissionPlan
    assert MissionPlan(task='trajectory').takeoff_feedforward is False
    MissionPlan(task='trajectory', takeoff_feedforward=True)
    with pytest.raises(ValueError, match='takeoff_feedforward'):
        MissionPlan(task='trajectory', takeoff_feedforward=1)
