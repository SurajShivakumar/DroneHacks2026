import xml.etree.ElementTree as ET
import pytest
from sim.yaw_diagnostic import build
from sim.gazebo_smoke import run


def test_fixture_changes_preserve_aircraft_and_course(tmp_path):
    source=tmp_path/'source.sdf'
    source.write_text('''<sdf><world><model name="aircraft"><link name="base_link"/></model>
    <model name="course"><static>true</static></model><model name="bootstrap_fixture">
    <link name="rail"><inertial><mass>1</mass></inertial></link>
    <plugin><parent_link>rail</parent_link><detach_topic>/diagnostic/bootstrap_detach</detach_topic></plugin>
    </model></world></sdf>''')
    output=tmp_path/'yaw.sdf'
    before=ET.parse(source)
    build(source,output)
    after=ET.parse(output)
    for name in ('aircraft','course'):
        path=f"world/model[@name='{name}']"
        assert ET.tostring(before.find(path))==ET.tostring(after.find(path))
    fixture=after.find("world/model[@name='bootstrap_fixture']")
    assert fixture.findtext('plugin/parent_link')=='yaw_cradle'
    assert fixture.find("joint[@name='diagnostic_yaw']").get('type')=='revolute'
    assert fixture.findtext("joint[@name='diagnostic_yaw']/axis/limit/velocity")=='0.4'
    with pytest.raises(FileExistsError):
        build(source,output)


def test_yaw_fixture_cannot_be_combined_with_flight_execution(tmp_path):
    with pytest.raises(ValueError,match='disarmed observation'):
        run(tmp_path,tmp_path,tmp_path/'scene',tmp_path/'out',
            bootstrap_fixture=True,execute_mission=True,diagnostic_yaw_rad=.5)
    assert not (tmp_path/'out').exists()


def test_ideal_depth_diagnostics_cannot_be_combined_with_flight_execution(tmp_path):
    with pytest.raises(ValueError,match='restricted to disarmed'):
        run(tmp_path,tmp_path,tmp_path/'scene',tmp_path/'out',execute_mission=True,record_ideal_depth=True)
    assert not (tmp_path/'out').exists()
