from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from sim.course_surface_appearance import build,read_binary_stl,surface_texture,write_uv_collada

ROOT=Path(__file__).resolve().parents[1]
NS={'c':'http://www.collada.org/2005/11/COLLADASchema'}


def inputs(output):
    return dict(source_model=ROOT/'artifacts/dronehacks_course_net_grid-v1/model.sdf',
        manifest=ROOT/'artifacts/course-v3-r6/scene_manifest.json',
        original_model=ROOT/'artifacts/course-v3-r6/models/dronehacks_course',
        scene=ROOT/'artifacts/dronehacks_course_net_grid-v1/world.sdf',output=output)


def test_seeded_albedo_is_deterministic_bounded_and_not_shared_between_pieces():
    a=surface_texture(42,[.7,.6,.5],'estimated_wood',size=128)
    np.testing.assert_array_equal(a,surface_texture(42,[.7,.6,.5],'estimated_wood',size=128))
    assert a.dtype==np.uint8 and a.shape==(128,128,3) and a.std()>2
    assert not np.array_equal(a,surface_texture(43,[.7,.6,.5],'estimated_wood',size=128))
    with pytest.raises(ValueError):surface_texture(42,[.7,.6,.5],'fiducial_grid',size=128)
    with pytest.raises(ValueError):surface_texture(42,[.7,.6,.5],'estimated_wood',strength=np.nan)


@pytest.mark.local_artifacts('artifacts/course-v3-r6/models/dronehacks_course/meshes/visual_03.stl')
def test_uv_export_preserves_winding_coordinates_normals_units_and_no_tiling(tmp_path):
    vertices,normals=read_binary_stl(ROOT/'artifacts/course-v3-r6/models/dronehacks_course/meshes/visual_03.stl')
    file=tmp_path/'wall.dae';report=write_uv_collada(file,vertices,normals)
    tree=ET.parse(file)
    values=lambda name:np.fromstring(tree.find(f".//c:source[@id='{name}']/c:float_array",NS).text,sep=' ')
    np.testing.assert_array_equal(values('positions').reshape(-1,3,3),vertices)
    np.testing.assert_array_equal(values('normals').reshape(-1,3),np.repeat(normals,3,axis=0))
    uv=values('uv').reshape(-1,2)
    assert np.all((uv>=0)&(uv<1)) and report['texture_span_m']==8
    assert tree.find('c:asset/c:unit',NS).get('meter')=='1'
    assert tree.findtext('c:asset/c:up_axis',namespaces=NS)=='Z_UP'
    indices=np.fromstring(tree.find('.//c:triangles/c:p',NS).text,sep=' ',dtype=int).reshape(-1,3)
    np.testing.assert_array_equal(indices,np.repeat(np.arange(len(vertices)*3)[:,None],3,axis=1))
    with pytest.raises(ValueError):write_uv_collada(tmp_path/'bad.dae',vertices,normals,texture_span_m=.01)


@pytest.mark.local_artifacts(
    'artifacts/dronehacks_course_net_grid-v1/model.sdf',
    'artifacts/dronehacks_course_net_grid-v1/world.sdf',
    'artifacts/course-v3-r6/scene_manifest.json',
    'artifacts/course-v3-r6/models/dronehacks_course',
)
def test_surface_variant_keeps_collision_nodes_net_visuals_sensors_and_world_settings(tmp_path):
    output=tmp_path/'dronehacks_course_surface_test'
    report=build(**inputs(output),seed=2026)
    assert report['all_collision_geometry_unchanged'] and report['all_visual_triangle_geometry_unchanged']
    assert report['original_collision_hashes']==report['variant_collision_hashes']
    assert report['renderer_validated'] is False and report['flight_completed'] is False
    assert len(report['modified_surfaces'])==11
    source=ET.parse(inputs(output)['source_model']).find('model/link')
    changed=ET.parse(output/'model.sdf').find('model/link')
    modified={r['visual'] for r in report['modified_surfaces']}
    assert len(source.findall('visual'))==len(changed.findall('visual'))
    for visual in source.findall('visual'):
        name=visual.get('name')
        if name not in modified:
            assert ET.tostring(visual)==ET.tostring(changed.find(f"visual[@name='{name}']"))
    for record in report['modified_surfaces']:
        before,_=read_binary_stl(inputs(output)['original_model']/'meshes'/f"visual_{record['visual'].split('_')[1]}.stl")
        tree=ET.parse(output/'meshes'/f"{record['visual']}.dae")
        after=np.fromstring(tree.find(".//c:source[@id='positions']/c:float_array",NS).text,sep=' ').reshape(-1,3,3)
        np.testing.assert_array_equal(before,after)
    old_world=ET.parse(inputs(output)['scene']);new_world=ET.parse(output/'world.sdf')
    assert [ET.tostring(n) for n in old_world.findall('.//sensor')]==[ET.tostring(n) for n in new_world.findall('.//sensor')]
    assert [ET.tostring(n) for n in old_world.findall('.//plugin')]==[ET.tostring(n) for n in new_world.findall('.//plugin')]
    assert new_world.findtext('world/include/uri')=='model://'+output.name
    repeated=build(**inputs(tmp_path/'dronehacks_course_surface_repeat'),seed=2026)
    assert repeated['modified_surfaces']==report['modified_surfaces']
    with pytest.raises(FileExistsError):build(**inputs(output))
