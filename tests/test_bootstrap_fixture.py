from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

from sim.bootstrap_fixture import build


@pytest.mark.local_artifacts('artifacts/gazebo-proxy-v8/world.sdf')
def test_fixture_keeps_aircraft_motors_and_sensors(tmp_path):
    source = Path('artifacts/gazebo-proxy-v8/world.sdf')
    old = ET.parse(source).find("world/model[@name='dronehacks_x500_proxy']")
    output = tmp_path / 'fixture.sdf'
    build(source, output)
    world = ET.parse(output).find('world')
    drone = world.find("model[@name='dronehacks_x500_proxy']")
    assert len(drone.findall('plugin')) == len(old.findall('plugin'))
    assert len(drone.findall('link')) == len(old.findall('link'))
    assert len(drone.findall('.//collision')) == len(old.findall('.//collision'))
    assert drone.find("link/sensor[@type='navsat']") is None
    fixture = world.find("model[@name='bootstrap_fixture']")
    assert fixture.find('joint/parent').text == 'world'
    assert not fixture.findall('.//sensor')
    assert fixture.find("plugin[@name='gz::sim::systems::DetachableJoint']/child_model").text == drone.get('name')
