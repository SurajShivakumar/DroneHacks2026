from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from sim.course_arena_background import build,exterior_surfaces

ROOT=Path(__file__).resolve().parents[1]


def test_exterior_walls_face_inward_and_do_not_enter_legal_net_volume():
    surfaces=exterior_surfaces([-3,-3,0],[3,3,3.048])
    for surface in surfaces:
        centre=surface['vertices'].reshape(-1,3).mean(axis=0)
        assert np.dot(surface['normals'][0],np.array([0,0,1.5])-centre)>0
        assert surface['minimum_net_separation_m']>0
    with pytest.raises(ValueError):exterior_surfaces([-3,-3,0],[3,3,3.048],wall_offset_m=.1)


@pytest.mark.local_artifacts(
    'artifacts/dronehacks_course_net_grid-v1/model.sdf',
    'artifacts/dronehacks_course_net_grid-v1/world.sdf',
    'artifacts/course-v3-r6/scene_manifest.json',
)
def test_background_preserves_course_collisions_and_every_original_visual(tmp_path):
    source=ROOT/'artifacts/dronehacks_course_net_grid-v1/model.sdf'
    scene=ROOT/'artifacts/dronehacks_course_net_grid-v1/world.sdf'
    output=tmp_path/'dronehacks_course_arena_test'
    report=build(source,ROOT/'artifacts/course-v3-r6/scene_manifest.json',scene,output,texture_size=128)
    assert report['all_collision_geometry_unchanged'] and report['added_collisions']==0
    assert report['all_original_visuals_unchanged'] and report['all_added_surfaces_outside_legal_net_volume']
    assert report['renderer_validated'] is False and report['feature_coordinates_given_to_controller'] is False
    original=ET.parse(source).find('model/link');variant=ET.parse(output/'model.sdf').find('model/link')
    assert len(variant.findall('visual'))==len(original.findall('visual'))+5
    for visual in original.findall('visual'):
        assert ET.tostring(visual)==ET.tostring(variant.find(f"visual[@name='{visual.get('name')}']"))
    a=ET.parse(scene);b=ET.parse(output/'world.sdf')
    assert [ET.tostring(s) for s in a.findall('.//sensor')]==[ET.tostring(s) for s in b.findall('.//sensor')]
    with pytest.raises(FileExistsError):build(source,ROOT/'artifacts/course-v3-r6/scene_manifest.json',scene,output,texture_size=128)
