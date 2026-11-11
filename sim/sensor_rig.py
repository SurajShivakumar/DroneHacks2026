"""Diagnostic constrained stereo/IMU rig. Not a drone or course flight test."""
import argparse
import copy
from pathlib import Path
import xml.etree.ElementTree as ET


def build(source, output, *, checkerboard=True, height=.7, camera_pitch=0., down_rgb=False):
    if output.exists():
        raise FileExistsError('Preserve existing diagnostic scenes')
    tree = ET.parse(source)
    model = tree.find("world/model[@name='dronehacks_x500_proxy']")
    if model is None:
        raise ValueError('Expected proxy model missing')
    model.set('name', 'diagnostic_sensor_rig')
    pose = model.find('pose')
    values = pose.text.split()
    values[2] = str(height)
    pose.text = ' '.join(values)
    for child in list(model):
        if child.tag in ('joint', 'plugin') or (child.tag == 'link' and child.get('name') != 'base_link'):
            model.remove(child)
    base = model.find("link[@name='base_link']")
    for sensor in base.findall('sensor'):
        if sensor.get('type') in ('camera', 'depth_camera'):
            sensor_pose = sensor.find('pose')
            parts = sensor_pose.text.split()
            parts[4] = str(camera_pitch)
            sensor_pose.text = ' '.join(parts)
    if down_rgb:
        rgb = base.find("sensor[@name='rgb_camera']")
        if rgb is None:
            rgb = next(s for s in base.findall('sensor') if s.findtext('topic') == '/dronehacks/rgb')
        down = copy.deepcopy(rgb)
        down.set('name', 'diagnostic_down_rgb')
        down.find('topic').text = '/dronehacks/down_rgb'
        down.find('camera/camera_info_topic').text = '/dronehacks/down_rgb/camera_info'
        down.find('pose').text = '0 0 -.05 0 1.5707963267948966 0'
        base.append(down)
    for child in list(base):
        if child.tag in ('visual', 'collision'):
            base.remove(child)
    # A real physics joint constrains five degrees of freedom; motor effort
    # accelerates the remaining axis so IMU measurements include actual motion.
    joint = ET.SubElement(model, 'joint', name='diagnostic_slide', type='prismatic')
    ET.SubElement(joint, 'parent').text = 'world'
    ET.SubElement(joint, 'child').text = 'base_link'
    axis = ET.SubElement(joint, 'axis')
    ET.SubElement(axis, 'xyz').text = '0 1 0'
    limit = ET.SubElement(axis, 'limit')
    for key, value in (('lower', '-0.15'), ('upper', '0.15'), ('effort', '5'), ('velocity', '0.4')):
        ET.SubElement(limit, key).text = value
    plugin = ET.SubElement(model, 'plugin', filename='gz-sim-joint-position-controller-system',
                           name='gz::sim::systems::JointPositionController')
    for key, value in (('joint_name', 'diagnostic_slide'), ('topic', '/diagnostic/rig_position'),
                       ('p_gain', '20'), ('i_gain', '0'), ('d_gain', '8'),
                       ('cmd_max', '5'), ('cmd_min', '-5')):
        ET.SubElement(plugin, key).text = value
    # A visible calibration board tests the sensor/estimator plumbing in a
    # feature-rich diagnostic scene. It is never added to the competition scene.
    if not checkerboard:
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('xb') as handle:
            tree.write(handle, encoding='utf-8', xml_declaration=True)
        return
    world = tree.find('world')
    board = ET.SubElement(world, 'model', name='diagnostic_checkerboard')
    ET.SubElement(board, 'static').text = 'true'
    board_link = ET.SubElement(board, 'link', name='board')
    x0, y0 = float(values[0]), float(values[1])
    for row in range(8):
        for column in range(8):
            visual = ET.SubElement(board_link, 'visual', name=f'square_{row}_{column}')
            ET.SubElement(visual, 'pose').text = f'{x0 + (column - 3.5) * .08} {y0 + .85} {.96 + (row - 3.5) * .08} 0 0 0'
            box = ET.SubElement(ET.SubElement(visual, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = '.08 .005 .08'
            material = ET.SubElement(visual, 'material')
            color = '.95 .95 .95 1' if (row + column) % 2 else '.02 .02 .02 1'
            ET.SubElement(material, 'ambient').text = color
            ET.SubElement(material, 'diffuse').text = color
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as handle:
        tree.write(handle, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--no-checkerboard', action='store_true')
    parser.add_argument('--height', type=float, default=.7)
    parser.add_argument('--camera-pitch', type=float, default=0.)
    parser.add_argument('--down-rgb', action='store_true', help='extra diagnostic sensor absent from Sky Beta specs')
    args = parser.parse_args()
    build(args.source, args.output, checkerboard=not args.no_checkerboard,
          height=args.height, camera_pitch=args.camera_pitch, down_rgb=args.down_rgb)
