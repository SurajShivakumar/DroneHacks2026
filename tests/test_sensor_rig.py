import xml.etree.ElementTree as ET
from pathlib import Path
import pytest

from sim.sensor_rig import build

pytestmark = pytest.mark.local_artifacts('artifacts/gazebo-proxy-v8/world.sdf')


def test_rig_has_real_joint_and_no_motors_or_collisions(tmp_path):
    source = Path('artifacts/gazebo-proxy-v8/world.sdf')
    target = tmp_path / 'rig.sdf'
    build(source, target)
    rig = ET.parse(target).find("world/model[@name='diagnostic_sensor_rig']")
    assert len(rig.findall('link')) == 1
    assert rig.find("joint/parent").text == 'world'
    assert rig.find("joint/axis/limit/upper").text == '0.15'
    assert rig.find("link/sensor[@type='imu']") is not None
    assert rig.find('link/collision') is None
    assert len(rig.findall('plugin')) == 1


def test_course_rig_omits_checkerboard_and_marks_extra_camera(tmp_path):
    target = tmp_path / 'rig.sdf'
    build(Path('artifacts/gazebo-proxy-v8/world.sdf'), target,
          checkerboard=False, height=.3, camera_pitch=.35, down_rgb=True)
    tree = ET.parse(target)
    assert tree.find("world/model[@name='diagnostic_checkerboard']") is None
    sensors = tree.findall("world/model[@name='diagnostic_sensor_rig']/link/sensor")
    assert any(s.findtext('topic') == '/dronehacks/down_rgb' for s in sensors)
    down = next(s for s in sensors if s.findtext('topic') == '/dronehacks/down_rgb')
    assert down.findtext('camera/camera_info_topic') == '/dronehacks/down_rgb/camera_info'
    left = next(s for s in sensors if s.get('name') == 'left')
    assert float(left.findtext('pose').split()[4]) == .35
