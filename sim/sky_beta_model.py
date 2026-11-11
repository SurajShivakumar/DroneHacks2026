"""Build the supplied Sky Beta rigid body; unknown actuators remain disabled.

Uses CAD materials/masses and an explicit COM body origin. This is never a
renamed X500: no X500 masses, inertia, rotors or motor constants are inherited.
"""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


def numbers(values):
    return ' '.join(f'{float(v):.12g}' for v in values)


def check_properties(properties):
    mass = properties['cad_mass_kg_excluding_omissions']
    inertia = np.asarray(properties['inertia_about_com_flu_kg_m2'], float)
    com = np.asarray(properties['com_flu_m'], float)
    if (not np.isfinite(mass) or mass <= 0 or com.shape != (3,) or not np.isfinite(com).all()
            or inertia.shape != (3, 3) or not np.isfinite(inertia).all()
            or not np.allclose(inertia, inertia.T) or np.linalg.eigvalsh(inertia).min() <= 0):
        raise ValueError('positive finite mass and physical symmetric inertia required')
    eigenvalues = np.linalg.eigvalsh(inertia)
    if eigenvalues[-1] > eigenvalues[:2].sum()+1e-12:
        raise ValueError('principal moments violate the rigid-body triangle inequality')
    return mass, com, inertia


def color(name):
    if 'Motor' in name:
        return [.4, .08, .06, 1.]
    if 'OVONIC' in name:
        return [.8, .3, .04, 1.]
    if 'Camera' in name or 'Pixhawk 6C' in name:
        return [.35, .38, .4, 1.]
    return [.12, .14, .16, 1.]


def build(cad_dir, course_scene, output):
    properties = json.loads((cad_dir/'cad-properties.json').read_text())
    mass, com, inertia = check_properties(properties)
    output.mkdir(parents=True, exist_ok=False)
    shutil.copytree(cad_dir/'meshes', output/'meshes')
    parts = [part for part in properties['parts'] if part['mass_kg'] > 0]
    # SDF geometry/dynamics asset. No motor plugin is emitted before actuator
    # and propeller data are supplied, instead of silently using X500 values.
    sdf = ET.Element('sdf', version='1.9')
    model = ET.SubElement(sdf, 'model', name='sky_beta_cad')
    link = ET.SubElement(model, 'link', name='base_link')
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass').text = str(mass)
    tensor = ET.SubElement(inertial, 'inertia')
    for name, i, j in [('ixx',0,0), ('iyy',1,1), ('izz',2,2), ('ixy',0,1), ('ixz',0,2), ('iyz',1,2)]:
        ET.SubElement(tensor, name).text = str(inertia[i,j])
    for index, part in enumerate(parts):
        for category in ('visual', 'collision'):
            element = ET.SubElement(link, category, name=f'{category}_{index:02d}')
            if category == 'collision':
                bounds=np.asarray(part['bounds_flu_m'])
                ET.SubElement(element,'pose').text=numbers([*(bounds.mean(axis=0)-com),0,0,0])
                box=ET.SubElement(ET.SubElement(element,'geometry'),'box')
                ET.SubElement(box,'size').text=numbers(bounds[1]-bounds[0]+.001)
            else:
                ET.SubElement(element, 'pose').text = numbers([*-com, 0, 0, 0])
                mesh = ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh')
                ET.SubElement(mesh, 'uri').text = part['mesh']
                material = ET.SubElement(element, 'material')
                ET.SubElement(material, 'ambient').text = numbers(color(part['name']))
                ET.SubElement(material, 'diffuse').text = numbers(color(part['name']))
    ET.ElementTree(sdf).write(output/'model.sdf', encoding='utf-8', xml_declaration=True)
    config = ET.Element('model')
    ET.SubElement(config, 'name').text = 'Sky Beta CAD rigid body'
    ET.SubElement(config, 'version').text = '1'
    ET.SubElement(config, 'sdf', version='1.9').text = 'model.sdf'
    ET.ElementTree(config).write(output/'model.config', encoding='utf-8', xml_declaration=True)

    # Reuse the course and rendering infrastructure, replacing the entire proxy
    # body and its actuator/sensor blocks with the actual supplied geometry.
    tree = ET.parse(course_scene)
    root = tree.getroot()
    root.find('compiler').set('meshdir', str((course_scene.parent/'meshes').resolve()))
    world = root.find('worldbody')
    world.remove(world.find("body[@name='proxy_quad']"))
    for tag in ('actuator', 'sensor'):
        element = root.find(tag)
        if element is not None:
            root.remove(element)
    body = ET.SubElement(world, 'body', name='sky_beta', pos=numbers([-2.5146, .9144, .2+com[2]]),
                         quat=numbers([2**-.5, 0, 0, 2**-.5]))
    ET.SubElement(body, 'freejoint', name='sky_beta_free')
    ET.SubElement(body, 'inertial', pos='0 0 0', mass=str(mass),
                  fullinertia=numbers([inertia[0,0], inertia[1,1], inertia[2,2], inertia[0,1], inertia[0,2], inertia[1,2]]))
    assets = root.find('asset')
    for index, part in enumerate(parts):
        for collision in (False, True):
            name = f'sky_beta_{index:02d}'+('_collision' if collision else '')
            if collision:
                bounds=np.asarray(part['bounds_flu_m'])
                ET.SubElement(body,'geom',name=name,type='box',
                    pos=numbers(bounds.mean(axis=0)-com),size=numbers((bounds[1]-bounds[0])*.5+.0005),
                    mass='0',contype='1',conaffinity='1',group='3',rgba='0 0 0 0')
                continue
            path = part['mesh']
            # MuJoCo's binary STL reader caps a file at 200k facets. Preserve
            # dense CAD visuals exactly by exporting OBJ, not decimating them.
            source_mesh = output/path
            with source_mesh.open('rb') as stream:
                stream.seek(80)
                triangles = int.from_bytes(stream.read(4), 'little')
            if triangles > 190000:
                converted = source_mesh.with_suffix('.obj')
                trimesh.load_mesh(source_mesh).export(converted)
                path = str(converted.relative_to(output))
            ET.SubElement(assets, 'mesh', name=name, file=str((output/path).resolve()))
            ET.SubElement(body, 'geom', name=name, mesh=name, type='mesh', pos=numbers(-com), mass='0',
                          contype='1' if collision else '0', conaffinity='1' if collision else '0',
                          group='3' if collision else '0', rgba=numbers([0,0,0,0] if collision else color(part['name'])))
    tree.write(output/'course-sky-beta.xml', encoding='utf-8', xml_declaration=True)
    rotors = [{'cad_part_name': p['name'],
               'shaft_xy_from_com_flu_m': (np.array(p['com_flu_m'])[:2]-com[:2]).tolist(),
               'motor_bounds_from_com_flu_m': (np.array(p['bounds_flu_m'])-com).tolist()}
              for p in parts if 'Motor' in p['name']]
    contract = {'kind': 'sky_beta_cad_rigid_body_no_actuator_model',
                'cad_properties_sha256': sha256((cad_dir/'cad-properties.json').read_bytes()).hexdigest(),
                'mass_kg_excluding_omissions': mass, 'body_origin': 'CAD-derived center of mass',
                'body_axes': 'FLU; forward points toward CAD -Y camera',
                'cad_origin_from_body_flu_m': (-com).tolist(),
                'inertia_com_flu_kg_m2': inertia.tolist(), 'rotors': rotors,
                'omissions': properties['omissions'], 'missing': properties['dynamics_missing'],
                'motor_plugins_enabled': False, 'flight_model_ready': False,
                'hardware_qualified': False,
                'collision': 'one conservative component bounding box, padded 0.5mm per side, in both engines',
                'collision_correction': 'Primitive contacts avoid the preserved Gazebo ODE trimesh contact-bucket overflow from hull meshes'}
    (output/'airframe-contract.json').write_text(json.dumps(contract, indent=2), encoding='utf-8')
    return contract


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cad', type=Path, required=True)
    parser.add_argument('--course', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.cad, args.course, args.output), indent=2))
