"""Explicit net appearance variant; preserves every original course collision.

The published OBJ represents netting as featureless sheets. This alternative
renders estimated strands at the same bounds. It is a separate appearance
scenario, not evidence that the real net has this pitch or visual observability.
"""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def digest_node(node):
    return sha256(ET.tostring(node)).hexdigest()


def build(course, scene, output, pitch=.05, thickness=.003):
    if not np.isfinite([pitch,thickness]).all() or not .01<=pitch<=.2 or not 0<thickness<pitch/3:
        raise ValueError('reviewed finite estimated net dimensions required')
    output.mkdir(parents=True,exist_ok=False)
    metadata=json.loads((course/'scene_manifest.json').read_text())
    tree=ET.parse(course/'models/dronehacks_course/model.sdf')
    model=tree.find('model'); model.set('name',output.name)
    link=model.find('link'); before=[digest_node(c) for c in link.findall('collision')]
    nets={f'piece_{i:02d}':p['name'] for i,p in enumerate(metadata['visual_pieces']) if p['net']}
    for visual in list(link.findall('visual')):
        if visual.get('name') in nets: link.remove(visual)
    count=0
    for name in nets.values():
        bounds=metadata['geometry']['groups'][name]
        low,high=np.array(bounds['min']),np.array(bounds['max'])
        centre=(low+high)/2; extent=high-low; normal=int(np.argmin(extent))
        axes=[i for i in range(3) if i!=normal]
        for along,across in (axes,axes[::-1]):
            for coordinate in np.linspace(low[across],high[across],int(np.ceil(extent[across]/pitch))+1):
                position=centre.copy(); position[across]=coordinate
                size=np.full(3,thickness); size[along]=extent[along]
                visual=ET.SubElement(link,'visual',name=f'estimated_net_strand_{count}')
                ET.SubElement(visual,'pose').text=' '.join(map(str,[*position,0,0,0]))
                geometry=ET.SubElement(visual,'geometry')
                ET.SubElement(ET.SubElement(geometry,'box'),'size').text=' '.join(map(str,size))
                material=ET.SubElement(visual,'material')
                for tag in ('ambient','diffuse'): ET.SubElement(material,tag).text='.06 .06 .06 1'
                ET.SubElement(visual,'cast_shadows').text='false'
                count+=1
    after=[digest_node(c) for c in link.findall('collision')]
    if before!=after: raise RuntimeError('appearance variant changed collision geometry')
    tree.write(output/'model.sdf',encoding='utf-8',xml_declaration=True)
    (output/'model.config').write_text('<model><name>Estimated net strand appearance variant</name><version>1</version><sdf version="1.6">model.sdf</sdf></model>')
    world=ET.parse(scene)
    includes=[node for node in world.findall('world/include/uri') if node.text=='model://dronehacks_course']
    if len(includes)!=1: raise ValueError('expected one original published course include')
    includes[0].text='model://'+output.name
    world.write(output/'world.sdf',encoding='utf-8',xml_declaration=True)
    report={'kind':'estimated_net_appearance_variant_not_original_asset_or_qualification',
        'source_course_sha256':sha256((course/'models/dronehacks_course/model.sdf').read_bytes()).hexdigest(),
        'source_scene_sha256':sha256(scene.read_bytes()).hexdigest(),
        'generator_sha256':sha256(Path(__file__).read_bytes()).hexdigest(),
        'estimated_pitch_m':pitch,'estimated_strand_thickness_m':thickness,'visual_strands':count,
        'original_collision_hashes':before,'variant_collision_hashes':after,
        'all_collision_geometry_unchanged':before==after,
        'limitations':'Net appearance is estimated, not measured. Preserve the original feature-poor scenario and report results separately. No fiducials, truth odometry, altered obstacles, calibration, autopilot settings or flight algorithms are added.'}
    (output/'appearance-contract.json').write_text(json.dumps(report,indent=2))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('course','scene','output'): parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--pitch',type=float,default=.05)
    parser.add_argument('--thickness',type=float,default=.003)
    args=parser.parse_args()
    print(json.dumps(build(args.course,args.scene,args.output,args.pitch,args.thickness),indent=2))
