"""Offline estimated exterior-arena appearance sensitivity, never navigation.

Adds visual-only stochastic plaster/concrete surfaces outside the published
net volume. Every source course visual/collision stays byte-identical after
XML parsing. No venue dimensions or material measurements are available.
"""
import argparse
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import PIL
from sim.course_surface_appearance import node_hash,surface_texture,write_uv_collada


def exterior_surfaces(net_min,net_max,*,wall_offset_m=1.45,wall_height_m=3.5,floor_source_min_z=-.03):
    low=np.asarray(net_min,float);high=np.asarray(net_max,float)
    if (low.shape!=(3,) or high.shape!=(3,) or not np.isfinite([*low,*high,wall_offset_m,wall_height_m,floor_source_min_z]).all()
            or np.any(low>=high) or not .5<=wall_offset_m<=5 or not high[2]<wall_height_m<=8):
        raise ValueError('finite legal net bounds and separated exterior dimensions required')
    x0,x1=low[0]-wall_offset_m,high[0]+wall_offset_m
    y0,y1=low[1]-wall_offset_m,high[1]+wall_offset_m
    z0=min(low[2],floor_source_min_z)-.01;z1=wall_height_m
    corners=[('west',[[x0,y0,z0],[x0,y1,z0],[x0,y1,z1],[x0,y0,z1]],0,-1),
             ('east',[[x1,y1,z0],[x1,y0,z0],[x1,y0,z1],[x1,y1,z1]],0,1),
             ('south',[[x0,y0,z0],[x0,y0,z1],[x1,y0,z1],[x1,y0,z0]],1,-1),
             ('north',[[x0,y1,z0],[x1,y1,z0],[x1,y1,z1],[x0,y1,z1]],1,1),
             ('floor',[[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0]],2,-1)]
    records=[]
    for name,vertices,axis,side in corners:
        points=np.asarray(vertices,float);triangles=points[[[0,1,2],[0,2,3]]]
        normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);normals/=np.linalg.norm(normals,axis=1)[:,None]
        boundary=high[axis] if side>0 else low[axis]
        if not np.all(side*(points[:,axis]-boundary)>0):raise RuntimeError('added surface entered legal net volume')
        if name=='floor' and not np.all(points[:,2]<floor_source_min_z):raise RuntimeError('background floor is not below original floor')
        records.append(dict(name=name,vertices=triangles,normals=normals,separation_axis=axis,separation_side=side,
                            minimum_net_separation_m=float(np.min(side*(points[:,axis]-boundary)))))
    return records


def build(source_model,manifest,scene,output,*,seed=2026,wall_offset_m=1.45,wall_height_m=3.5,strength=2.,texture_size=1024):
    source_model,manifest,scene,output=map(Path,(source_model,manifest,scene,output))
    if (not output.name.startswith('dronehacks_course') or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in output.name)):
        raise ValueError('portable new course-model name required')
    if type(seed) is not int or not 0<=seed<2**64:raise ValueError('bounded deterministic seed required')
    data=json.loads(manifest.read_text());groups=data['geometry']['groups']
    nets=[value for name,value in groups.items() if name.startswith('Net')]
    floor=[value for name,value in groups.items() if name.lower().startswith('floor ')]
    if len(nets)<4 or not floor:raise ValueError('published net and floor geometry required')
    low=np.min([g['min'] for g in nets],axis=0);high=np.max([g['max'] for g in nets],axis=0)
    source_floor_min=float(min(g['min'][2] for g in floor))
    surfaces=exterior_surfaces(low,high,wall_offset_m=wall_offset_m,wall_height_m=wall_height_m,floor_source_min_z=source_floor_min)
    tree=ET.parse(source_model);model=tree.find('model')
    if model is None or model.findtext('static')!='true' or len(model.findall('link'))!=1:raise ValueError('single-link static source course required')
    link=model.find('link');source_name=model.get('name')
    visuals={v.get('name'):node_hash(v) for v in link.findall('visual')}
    if len(visuals)!=len(link.findall('visual')):raise ValueError('unique source visual names required')
    collisions=[node_hash(c) for c in model.findall('.//collision')]
    world=ET.parse(scene);original_world=deepcopy(world.getroot())
    include=[n for n in world.findall('world/include/uri') if n.text=='model://'+source_name]
    if len(include)!=1:raise ValueError('exactly the selected source course include required')
    # Validate material settings before creating an exclusive evidence directory.
    surface_texture(seed,[.55,.55,.54],'estimated_concrete',size=texture_size,strength=strength)
    output.mkdir(parents=True,exist_ok=False);(output/'meshes').mkdir();(output/'materials/textures').mkdir(parents=True)
    records=[]
    for surface in surfaces:
        name=surface['name'];stem='estimated_exterior_'+name
        mesh=output/'meshes'/f'{stem}.dae'
        span=float(np.ceil(max(np.ptp(surface['vertices'].reshape(-1,3),axis=0))+1))
        uv=write_uv_collada(mesh,surface['vertices'],surface['normals'],texture_span_m=span)
        derived=int.from_bytes(sha256(f'{seed}:estimated-exterior:{name}'.encode()).digest()[:8],'little')
        rgb=[.48,.48,.47] if name=='floor' else [.62,.61,.59]
        texture=output/'materials/textures'/f'{stem}.png'
        Image.fromarray(surface_texture(derived,rgb,'estimated_concrete',size=texture_size,strength=strength)).save(texture)
        visual=ET.SubElement(link,'visual',name=stem)
        ET.SubElement(ET.SubElement(ET.SubElement(visual,'geometry'),'mesh'),'uri').text=f'model://{output.name}/meshes/{mesh.name}'
        material=ET.SubElement(visual,'material')
        ET.SubElement(material,'ambient').text='.2 .2 .2 1';ET.SubElement(material,'diffuse').text='1 1 1 1'
        metal=ET.SubElement(ET.SubElement(material,'pbr'),'metal')
        ET.SubElement(metal,'albedo_map').text=f'model://{output.name}/materials/textures/{texture.name}'
        ET.SubElement(metal,'roughness').text='.9';ET.SubElement(metal,'metalness').text='0'
        ET.SubElement(visual,'cast_shadows').text='false'
        records.append(dict(name=name,seed=derived,vertices_m=surface['vertices'].tolist(),normals=surface['normals'].tolist(),
            geometry_sha256=sha256(surface['vertices'].astype('<f8').tobytes()).hexdigest(),
            mesh_sha256=sha256(mesh.read_bytes()).hexdigest(),texture_sha256=sha256(texture.read_bytes()).hexdigest(),
            separation_axis=surface['separation_axis'],separation_side=surface['separation_side'],
            minimum_net_separation_m=surface['minimum_net_separation_m'],albedo_base_rgb=rgb,uv=uv))
    if any(node_hash(link.find(f"visual[@name='{name}']"))!=digest for name,digest in visuals.items()):raise RuntimeError('existing visual/net modified')
    after_collisions=[node_hash(c) for c in model.findall('.//collision')]
    if collisions!=after_collisions:raise RuntimeError('source collision changed or background collision added')
    model.set('name',output.name);tree.getroot().set('version','1.7')
    tree.write(output/'model.sdf',encoding='utf-8',xml_declaration=True)
    (output/'model.config').write_text('<model><name>Estimated exterior arena sensitivity</name><version>1</version><sdf version="1.7">model.sdf</sdf></model>')
    include[0].text='model://'+output.name
    check=deepcopy(world.getroot())
    next(n for n in check.findall('world/include/uri') if n.text=='model://'+output.name).text='model://'+source_name
    if ET.tostring(check)!=ET.tostring(original_world):raise RuntimeError('camera, physics, lighting, plugins or world settings changed')
    world.write(output/'world.sdf',encoding='utf-8',xml_declaration=True)
    helper=Path(__file__).with_name('course_surface_appearance.py')
    report=dict(kind='estimated_exterior_background_sensitivity_not_venue_or_flight_qualification',
        generator_sha256=sha256(Path(__file__).read_bytes()).hexdigest(),surface_helper_sha256=sha256(helper.read_bytes()).hexdigest(),
        source_model_sha256=sha256(source_model.read_bytes()).hexdigest(),source_manifest_sha256=sha256(manifest.read_bytes()).hexdigest(),
        source_scene_sha256=sha256(scene.read_bytes()).hexdigest(),source_net_bounds_min_m=low.tolist(),source_net_bounds_max_m=high.tolist(),
        source_floor_min_z_m=source_floor_min,seed=seed,wall_offset_from_net_m=wall_offset_m,wall_height_m=wall_height_m,
        material_assumptions=dict(kind='procedural estimated concrete/plaster',contrast_scale=strength,albedo_std=.025*strength,
            clipping_std=3.,texture_pixels_per_axis=texture_size,ambient_rgb=[.2]*3,roughness=.9,metalness=0.,casts_shadows=False,
            note='Unmeasured venue geometry/material assumptions; no photographs or mapped landmarks'),
        numpy_version=np.__version__,pillow_version=PIL.__version__,added_visual_surfaces=records,
        original_collision_hashes=collisions,variant_collision_hashes=after_collisions,all_collision_geometry_unchanged=True,
        original_visual_hashes=visuals,all_original_visuals_unchanged=True,all_added_surfaces_outside_legal_net_volume=True,
        background_floor_below_source_floor=True,sensors_calibration_physics_lighting_plugins_unchanged=True,
        added_collisions=0,renderer_validated=False,flight_completed=False,feature_coordinates_given_to_controller=False,
        limitations='Synthetic exterior appearance only; original course remains unchanged. No real venue survey, renderer/UV proof, estimator qualification or flight result. Preserve and report original/net/surface failures separately.')
    (output/'appearance-contract.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    shutil.copy2(Path(__file__),output/'generator.py');shutil.copy2(helper,output/'surface_helper.py')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source-model','manifest','scene','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--seed',type=int,default=2026);parser.add_argument('--wall-offset-m',type=float,default=1.45)
    parser.add_argument('--wall-height-m',type=float,default=3.5);parser.add_argument('--strength',type=float,default=2.)
    args=parser.parse_args();report=build(args.source_model,args.manifest,args.scene,args.output,seed=args.seed,
        wall_offset_m=args.wall_offset_m,wall_height_m=args.wall_height_m,strength=args.strength)
    print(json.dumps({k:v for k,v in report.items() if k not in ('added_visual_surfaces','original_visual_hashes')},indent=2))
