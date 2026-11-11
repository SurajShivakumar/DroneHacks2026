from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from sim.course_net_visual import build


@pytest.mark.local_artifacts(
    'artifacts/course-v3-r6/scene_manifest.json',
    'artifacts/course-v3-r6/models/dronehacks_course/model.sdf',
    'artifacts/sky-beta-estimated-v4/overhead-fixture-world.sdf',
)
def test_net_appearance_variant_preserves_collision_mesh_and_published_walls(tmp_path):
    root=Path(__file__).resolve().parents[1]
    output=tmp_path/'net_variant'
    report=build(root/'artifacts/course-v3-r6',root/'artifacts/sky-beta-estimated-v4/overhead-fixture-world.sdf',output)
    original=ET.parse(root/'artifacts/course-v3-r6/models/dronehacks_course/model.sdf').find('model/link')
    variant=ET.parse(output/'model.sdf').find('model/link')
    assert report['all_collision_geometry_unchanged']
    assert ET.tostring(original.find('collision'))==ET.tostring(variant.find('collision'))
    old={v.get('name'):ET.tostring(v) for v in original.findall('visual')}
    retained={v.get('name'):ET.tostring(v) for v in variant.findall('visual') if not v.get('name').startswith('estimated_net')}
    assert len(retained)==28 and all(old[k]==v for k,v in retained.items())
    assert report['visual_strands']>500
