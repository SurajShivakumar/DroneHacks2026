"""Add bounded physical yaw motion to the disarmed bootstrap fixture only."""
import argparse
from math import isfinite
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET


def build(source, output, rate_rad_s=.4):
    if not isfinite(rate_rad_s) or not .05<=rate_rad_s<=.4:
        raise ValueError('diagnostic yaw rate must be 0.05..0.4 rad/s')
    tree = ET.parse(source)
    fixture = tree.find("world/model[@name='bootstrap_fixture']")
    if fixture is None or fixture.find("link[@name='rail']") is None:
        raise ValueError('existing bootstrap rail required')
    detach = next((p for p in fixture.findall('plugin')
                   if p.findtext('detach_topic') == '/diagnostic/bootstrap_detach'), None)
    if detach is None or detach.findtext('parent_link') != 'rail':
        raise ValueError('reviewed rail detach contract required')
    if fixture.find("joint[@name='diagnostic_yaw']") is not None:
        raise ValueError('yaw fixture already exists')
    cradle = ET.SubElement(fixture, 'link', name='yaw_cradle')
    # Extra mass belongs to the diagnostic fixture, never to the aircraft.
    cradle.append(deepcopy(fixture.find("link[@name='rail']/inertial")))
    joint = ET.SubElement(fixture, 'joint', name='diagnostic_yaw', type='revolute')
    ET.SubElement(joint, 'parent').text = 'rail'
    ET.SubElement(joint, 'child').text = 'yaw_cradle'
    axis = ET.SubElement(joint, 'axis')
    ET.SubElement(axis, 'xyz').text = '0 0 1'
    limit = ET.SubElement(axis, 'limit')
    for key, value in [('lower', '-1.7'), ('upper', '1.7'),
                       ('effort', '0.3'), ('velocity', str(rate_rad_s))]:
        ET.SubElement(limit, key).text = value
    plugin = ET.SubElement(fixture, 'plugin',
        filename='gz-sim-joint-position-controller-system',
        name='gz::sim::systems::JointPositionController')
    for key, value in [('joint_name', 'diagnostic_yaw'),
                       ('topic', '/diagnostic/rig_yaw_position'),
                       ('p_gain', '0.3'), ('d_gain', '0.1'), ('i_gain', '0'),
                       ('cmd_max', '0.3'), ('cmd_min', '-0.3')]:
        ET.SubElement(plugin, key).text = value
    detach.find('parent_link').text = 'yaw_cradle'
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as stream:
        tree.write(stream, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--rate-rad-s', type=float, default=.4)
    args = parser.parse_args()
    build(args.source, args.output,args.rate_rad_s)
