"""Physical preflight motion fixture for a DISARMED proxy aircraft.

The fixture slides the aircraft using a real physics joint, then detaches it.
It supplies no odometry, GNSS or flight commands. This is test setup, never
part of an onboard flight algorithm or evidence of hardware qualification.
"""
import argparse
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET


def value(parent, name, text):
    ET.SubElement(parent, name).text = str(text)


def build(source, output, camera_pitch=.35, model_name='dronehacks_x500_proxy'):
    if output.exists():
        raise FileExistsError('Preserve prior fixture scenes')
    tree = ET.parse(source)
    world = tree.find('world')
    drone = world.find(f"model[@name='{model_name}']")
    if drone is None:
        raise ValueError('Expected intact proxy aircraft')
    for sensor in drone.findall("link[@name='base_link']/sensor"):
        if sensor.get('type') in ('camera', 'depth_camera'):
            pose = sensor.find('pose')
            parts = pose.text.split()
            parts[4] = str(camera_pitch)
            pose.text = ' '.join(parts)
    fixture = ET.SubElement(world, 'model', name='bootstrap_fixture')
    fixture.append(deepcopy(drone.find('pose')))
    rail = ET.SubElement(fixture, 'link', name='rail')
    inertial = ET.SubElement(rail, 'inertial')
    value(inertial, 'mass', '1')
    inertia = ET.SubElement(inertial, 'inertia')
    for name in ('ixx', 'iyy', 'izz'):
        value(inertia, name, '.01')
    joint = ET.SubElement(fixture, 'joint', name='diagnostic_slide', type='prismatic')
    value(joint, 'parent', 'world')
    value(joint, 'child', 'rail')
    axis = ET.SubElement(joint, 'axis')
    # The fixture root inherits the drone's pi/2 yaw: local +X maps to world +Y, along the course corridor.
    value(axis, 'xyz', '1 0 0')
    limits = ET.SubElement(axis, 'limit')
    for name, number in (('lower', '-.15'), ('upper', '.15'), ('effort', '5'), ('velocity', '.4')):
        value(limits, name, number)
    control = ET.SubElement(fixture, 'plugin', filename='gz-sim-joint-position-controller-system',
                            name='gz::sim::systems::JointPositionController')
    for name, text in (('joint_name', 'diagnostic_slide'), ('topic', '/diagnostic/rig_position'),
                       ('p_gain', '20'), ('d_gain', '8'), ('i_gain', '0'), ('cmd_max', '5'), ('cmd_min', '-5')):
        value(control, name, text)
    detach = ET.SubElement(fixture, 'plugin', filename='gz-sim-detachable-joint-system',
                           name='gz::sim::systems::DetachableJoint')
    for name, text in (('parent_link', 'rail'), ('child_model', model_name),
                       ('child_link', 'base_link'), ('detach_topic', '/diagnostic/bootstrap_detach'),
                       ('output_topic', '/diagnostic/bootstrap_state')):
        value(detach, name, text)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as stream:
        tree.write(stream, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    build(args.source, args.output)
