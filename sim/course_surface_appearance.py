"""Separate estimated stochastic surface-albedo sensitivity scenario.

Does not start a simulator. Source STL triangle coordinates/winding are kept
exactly, with new dominant-plane UVs in COLLADA (metres, Z up). All collision
nodes and every unselected visual, including existing net strands, are kept.
Procedural wood/concrete labels and contrast are assumptions, not venue data.
No fiducials, feature-position map, normals/displacement or lighting edits.
"""
import argparse
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import PIL


def node_hash(node):return sha256(ET.tostring(node)).hexdigest()


def read_binary_stl(path):
    raw=path.read_bytes()
    if len(raw)<84:raise ValueError('binary STL header missing')
    count=struct.unpack_from('<I',raw,80)[0]
    if count<1 or len(raw)!=84+50*count:raise ValueError('complete binary STL triangles required')
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
    facets=np.frombuffer(raw,dtype=dtype,count=count,offset=84)
    vertices=facets['vertices'].astype(float);normals=facets['normal'].astype(float)
    if not np.isfinite(vertices).all() or not np.isfinite(normals).all():raise ValueError('finite STL geometry required')
    if np.any(np.linalg.norm(np.cross(vertices[:,1]-vertices[:,0],vertices[:,2]-vertices[:,0]),axis=1)<1e-12):
        raise ValueError('degenerate visual triangle cannot receive a planar UV')
    return vertices,normals


def write_uv_collada(path,vertices,normals,*,texture_span_m=8.):
    """Explicit per-corner UVs without vertex movement, merging or remeshing."""
    vertices=np.asarray(vertices,float); normals=np.asarray(normals,float)
    if vertices.ndim!=3 or vertices.shape[1:]!=(3,3) or normals.shape!=(len(vertices),3) or not np.isfinite(vertices).all() or not np.isfinite(normals).all():
        raise ValueError('finite triangular vertices and facet normals required')
    extent=np.ptp(vertices.reshape(-1,3),axis=0)
    if not np.isfinite(texture_span_m) or texture_span_m<=max(extent):
        raise ValueError('texture span must exceed the whole piece extent to avoid tiling')
    low=vertices.reshape(-1,3).min(axis=0);uv=[]
    for triangle,normal in zip(vertices,normals):
        normal_axis=int(np.argmax(np.abs(normal)));axes=[a for a in range(3) if a!=normal_axis]
        uv.extend((triangle[:,axes]-low[axes])/texture_span_m)
    positions=vertices.reshape(-1,3);facet_normals=np.repeat(normals,3,axis=0);uv=np.asarray(uv)
    root=ET.Element('COLLADA',xmlns='http://www.collada.org/2005/11/COLLADASchema',version='1.4.1')
    asset=ET.SubElement(root,'asset')
    for tag in ('created','modified'):ET.SubElement(asset,tag).text='1970-01-01T00:00:00Z'
    ET.SubElement(asset,'unit',name='meter',meter='1');ET.SubElement(asset,'up_axis').text='Z_UP'
    mesh=ET.SubElement(ET.SubElement(ET.SubElement(root,'library_geometries'),'geometry',id='surface'),'mesh')
    for name,array,params in (('positions',positions,('X','Y','Z')),('normals',facet_normals,('X','Y','Z')),('uv',uv,('S','T'))):
        source=ET.SubElement(mesh,'source',id=name)
        ET.SubElement(source,'float_array',id=name+'-array',count=str(array.size)).text=' '.join(format(float(v),'.17g') for v in array.ravel())
        access=ET.SubElement(ET.SubElement(source,'technique_common'),'accessor',source='#'+name+'-array',count=str(len(array)),stride=str(len(params)))
        for parameter in params:ET.SubElement(access,'param',name=parameter,type='float')
    vertex=ET.SubElement(mesh,'vertices',id='vertices');ET.SubElement(vertex,'input',semantic='POSITION',source='#positions')
    triangles=ET.SubElement(mesh,'triangles',count=str(len(vertices)))
    for offset,(semantic,source) in enumerate((('VERTEX','#vertices'),('NORMAL','#normals'),('TEXCOORD','#uv'))):
        attributes=dict(semantic=semantic,source=source,offset=str(offset))
        if semantic=='TEXCOORD':attributes['set']='0'
        ET.SubElement(triangles,'input',**attributes)
    ET.SubElement(triangles,'p').text=' '.join(str(i) for i in np.repeat(np.arange(len(positions)),3))
    visual_scene=ET.SubElement(ET.SubElement(root,'library_visual_scenes'),'visual_scene',id='scene')
    node=ET.SubElement(visual_scene,'node',id='surface-node');ET.SubElement(node,'instance_geometry',url='#surface')
    ET.SubElement(ET.SubElement(root,'scene'),'instance_visual_scene',url='#scene')
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    return dict(triangles=len(vertices),uv_min=uv.min(axis=0).tolist(),uv_max=uv.max(axis=0).tolist(),
                texture_span_m=texture_span_m,unit_m=1,up_axis='Z_UP')


def surface_texture(seed,base_rgb,kind,*,size=1024,strength=1.):
    """Band-limited stochastic albedo; not photographs or mapped landmarks."""
    base=np.asarray(base_rgb,float)
    if type(seed) is not int or not 0<=seed<2**64 or base.shape!=(3,) or not np.isfinite(base).all() or np.any((base<0)|(base>1)):
        raise ValueError('bounded seed and RGB albedo required')
    if kind not in ('estimated_wood','estimated_concrete') or type(size) is not int or not 64<=size<=2048 or not np.isfinite(strength) or not 0<=strength<=2:
        raise ValueError('bounded procedural material settings required')
    rng=np.random.default_rng(seed)
    def noise(height,width):
        coarse=rng.normal(size=(height,width)).astype(np.float32)
        field=np.asarray(Image.fromarray(coarse).resize((size,size),resample=Image.Resampling.BICUBIC),dtype=float)
        return (field-field.mean())/max(field.std(),1e-12)
    if kind=='estimated_wood':
        field=.45*noise(24,24)+.4*noise(24,384)+.15*noise(128,512);sigma=.045
    else:
        field=.55*noise(32,32)+.3*noise(128,128)+.15*noise(512,512);sigma=.025
    field=(field-field.mean())/max(field.std(),1e-12)
    # Correlated color variation avoids highly colored artificial markers.
    albedo=np.clip(base[None,None,:]+strength*sigma*np.clip(field,-3,3)[:,:,None],0,1)
    return np.rint(albedo*255).astype(np.uint8)


def material_kind(piece):
    if piece['net']:return None
    name=piece['name'].lower()
    if name.startswith('floor '):return 'estimated_concrete'
    if 'wall' in name or 'divider' in name or name.startswith('barrier '):return 'estimated_wood'
    return None


def build(source_model,manifest,original_model,scene,output,*,seed=2026,strength=1.):
    source_model,manifest,original_model,scene,output=map(Path,(source_model,manifest,original_model,scene,output))
    if type(seed) is not int or not 0<=seed<2**64 or not np.isfinite(strength) or not 0<=strength<=2:raise ValueError('bounded seed/contrast required')
    if not output.name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in output.name):
        raise ValueError('portable unique model directory name required')
    data=json.loads(manifest.read_text());tree=ET.parse(source_model);model=tree.find('model')
    if model is None or model.findtext('static')!='true':raise ValueError('static course model required')
    source_name=model.get('name');link=model.find('link')
    if len(model.findall('link'))!=1:raise ValueError('single existing course link required')
    world=ET.parse(scene)
    includes=[n for n in world.findall('world/include/uri') if n.text=='model://'+source_name]
    if len(includes)!=1:raise ValueError('scene must include exactly the selected source model')
    before_collisions=[node_hash(c) for c in link.findall('collision')]
    original_visuals={v.get('name'):deepcopy(v) for v in link.findall('visual')}
    if len(original_visuals)!=len(link.findall('visual')):raise ValueError('unique visual names required')
    output.mkdir(parents=True,exist_ok=False);(output/'meshes').mkdir();(output/'materials/textures').mkdir(parents=True)
    records=[]
    for index,piece in enumerate(data['visual_pieces']):
        kind=material_kind(piece)
        if kind is None:continue
        name=f'piece_{index:02d}';visual=link.find(f"visual[@name='{name}']")
        if visual is None:raise ValueError('selected source visual missing: '+name)
        mesh=visual.find('geometry/mesh');uri=mesh.find('uri') if mesh is not None else None
        expected='model://'+original_model.name+'/meshes/'+piece['file']
        if uri is None or uri.text!=expected or len(list(mesh))!=1:raise ValueError('unaltered source visual mesh required')
        source=original_model/'meshes'/piece['file']
        if not source.resolve().is_relative_to(original_model.resolve()):raise ValueError('mesh path escaped original model')
        vertices,normals=read_binary_stl(source)
        dae=output/'meshes'/f'{name}.dae';uv=write_uv_collada(dae,vertices,normals)
        derived=int.from_bytes(sha256(f'{seed}:{piece["name"]}'.encode()).digest()[:8],'little')
        image=surface_texture(derived,list(map(float,piece['color'])),kind,strength=strength)
        texture=output/'materials/textures'/f'{name}.png';Image.fromarray(image).save(texture)
        uri.text=f'model://{output.name}/meshes/{dae.name}'
        material=visual.find('material')
        if material is None:material=ET.SubElement(visual,'material')
        if material.find('pbr') is not None or material.find('script') is not None:raise ValueError('source already contains texture/material script')
        diffuse=material.find('diffuse')
        if diffuse is None:diffuse=ET.SubElement(material,'diffuse')
        diffuse.text='1 1 1 1' # Original Kd is already baked into the albedo pixels.
        metal=ET.SubElement(ET.SubElement(material,'pbr'),'metal')
        ET.SubElement(metal,'albedo_map').text=f'model://{output.name}/materials/textures/{texture.name}'
        ET.SubElement(metal,'metalness').text='0';ET.SubElement(metal,'roughness').text='.85'
        records.append(dict(name=piece['name'],visual=name,kind=kind,seed=derived,source_stl_sha256=sha256(source.read_bytes()).hexdigest(),
            triangle_coordinates_sha256=sha256(vertices.astype('<f8').tobytes()).hexdigest(),
            dae_sha256=sha256(dae.read_bytes()).hexdigest(),texture_sha256=sha256(texture.read_bytes()).hexdigest(),
            original_rgb=list(map(float,piece['color'])),texture_mean_rgb=(image.mean(axis=(0,1))/255).tolist(),uv=uv))
    if not records:raise ValueError('no selected surfaces changed; not an appearance scenario')
    changed={r['visual'] for r in records}
    for visual in link.findall('visual'):
        old=original_visuals[visual.get('name')]
        if visual.get('name') not in changed and ET.tostring(old)!=ET.tostring(visual):raise RuntimeError('unselected/net visual changed')
        if visual.get('name') in changed:
            check=deepcopy(visual);check.find('geometry/mesh/uri').text=old.findtext('geometry/mesh/uri')
            check.remove(check.find('material'))
            old_check=deepcopy(old);old_check.remove(old_check.find('material'))
            if ET.tostring(check)!=ET.tostring(old_check):raise RuntimeError('selected visual pose/geometry attributes changed')
    after_collisions=[node_hash(c) for c in link.findall('collision')]
    if before_collisions!=after_collisions:raise RuntimeError('collision geometry changed')
    model.set('name',output.name);tree.getroot().set('version','1.7')
    tree.write(output/'model.sdf',encoding='utf-8',xml_declaration=True)
    (output/'model.config').write_text('<model><name>Estimated stochastic surface appearance sensitivity</name><version>1</version><sdf version="1.7">model.sdf</sdf></model>')
    before_world=deepcopy(world.getroot());includes[0].text='model://'+output.name
    world.write(output/'world.sdf',encoding='utf-8',xml_declaration=True)
    # Compare world with only the course URI normalized, including cameras/plugins.
    comparison=deepcopy(world.getroot())
    next(n for n in comparison.findall('world/include/uri') if n.text=='model://'+output.name).text='model://'+source_name
    if ET.tostring(comparison)!=ET.tostring(before_world):raise RuntimeError('nonappearance world settings changed')
    obj=original_model/'meshes/dronehacks-course-v3.obj';mtl=obj.with_suffix('.mtl')
    material_lines=[line.split() for line in mtl.read_text().splitlines() if line.strip()]
    obj_uv_count=sum(bool(fields) and fields[0]=='vt' for fields in (line.split() for line in obj.read_text().splitlines()))
    report=dict(kind='estimated_procedural_surface_appearance_sensitivity_not_original_scene_or_qualification',
        source_model_sha256=sha256(source_model.read_bytes()).hexdigest(),source_manifest_sha256=sha256(manifest.read_bytes()).hexdigest(),
        source_scene_sha256=sha256(scene.read_bytes()).hexdigest(),source_obj_sha256=sha256(obj.read_bytes()).hexdigest(),source_mtl_sha256=sha256(mtl.read_bytes()).hexdigest(),
        generator_sha256=sha256(Path(__file__).read_bytes()).hexdigest(),seed=seed,contrast_scale=strength,
        numpy_version=np.__version__,pillow_version=PIL.__version__,
        original_material_scan=dict(obj_uv_coordinates=obj_uv_count,kd_materials=sum(fields[0]=='Kd' for fields in material_lines),
            image_texture_directives=[fields[0] for fields in material_lines if fields[0].startswith('map_') or fields[0] in ('bump','disp','decal')]),
        texture_assumptions=dict(atlas_span_m=8.,pixels_per_atlas_axis=1024,no_uv_tiling=True,
            wood_albedo_std_per_channel=.045*strength,concrete_albedo_std_per_channel=.025*strength,
            variation_clipped_at_std=3.,wood_noise_grids_hw=[[24,24],[24,384],[128,512]],wood_grid_weights=[.45,.4,.15],
            concrete_noise_grids_hw=[[32,32],[128,128],[512,512]],concrete_grid_weights=[.55,.3,.15],
            roughness=.85,metalness=0.,seed_derivation='first8bytes little-endian SHA256 of global-seed:published-piece-name',
            provenance='engineering assumptions; not venue measurements or photographs'),
        original_collision_hashes=before_collisions,variant_collision_hashes=after_collisions,
        all_collision_geometry_unchanged=True,all_visual_triangle_geometry_unchanged=True,
        unselected_visuals_unchanged=len(original_visuals)-len(records),modified_surfaces=records,
        sensors_calibration_physics_plugins_unchanged=True,renderer_validated=False,flight_completed=False,
        resource_paths='Register output parent plus original-model parent in GZ_SIM_RESOURCE_PATH. Original mesh URIs remain model://dronehacks_course/meshes/...; new assets use the separate model name.',
        limitations='Procedural estimated albedo only; no photos or venue material measurements, no fiducials or feature-coordinate maps. Geometry/UV/XML checks are offline. Gazebo renderer/UV/material visibility must be checked separately. Results belong to this sensitivity scenario only; retain original flat-color/net-grid failures.')
    (output/'appearance-contract.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    shutil.copy2(Path(__file__),output/'generator.py')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source-model','manifest','original-model','scene','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--seed',type=int,default=2026);parser.add_argument('--strength',type=float,default=1.)
    args=parser.parse_args();result=build(args.source_model,args.manifest,args.original_model,args.scene,args.output,seed=args.seed,strength=args.strength)
    print(json.dumps({k:v for k,v in result.items() if k not in ('modified_surfaces','original_collision_hashes','variant_collision_hashes')},indent=2))
