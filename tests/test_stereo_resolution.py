from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from sim.stereo_resolution import build,load_yaml,mono_sensors


SCENE=Path('artifacts/dronehacks_course_net_grid-v1/world.sdf')
CONFIG=Path('artifacts/vio-feature-budget-v1/estimator_config.yaml')

pytestmark = pytest.mark.local_artifacts(
    str(SCENE), str(CONFIG),
    'artifacts/vio-feature-budget-v1/kalibr_imucam_chain.yaml',
)


def test_only_stereo_dimensions_and_corresponding_calibration_change(tmp_path):
    out=tmp_path/'native';report=build(SCENE,CONFIG,out)
    assert report['only_four_sdf_dimension_texts_changed']
    assert report['source_canonical_sdf_sha256']==report['reverted_variant_canonical_sdf_sha256']
    assert (out/'vio'/CONFIG.name).read_bytes()==CONFIG.read_bytes()
    original=load_yaml(CONFIG.parent/'kalibr_imucam_chain.yaml');changed=load_yaml(out/'vio/kalibr_imucam_chain.yaml')
    for name in ('cam0','cam1'):
        assert changed[name]['resolution']==[1280,800]
        np.testing.assert_equal(changed[name]['intrinsics'],np.asarray(original[name]['intrinsics'])*2)
        assert changed[name]['T_imu_cam']==original[name]['T_imu_cam']
        assert changed[name]['distortion_coeffs']==original[name]['distortion_coeffs']
    pair=mono_sensors(ET.parse(out/'world.sdf'),'sky_beta_estimated')
    assert [s.findtext('camera/image/width') for s in pair]==['1280','1280']
    assert report['render_runtime_validated'] is False and report['flight_ready'] is False
    with pytest.raises(FileExistsError):build(SCENE,CONFIG,out)


def test_generated_calibration_loads_through_actual_opencv_consumer(tmp_path):
    cv2=pytest.importorskip('cv2')
    out=tmp_path/'consumer';build(SCENE,CONFIG,out)
    storage=cv2.FileStorage(str(out/'vio/kalibr_imucam_chain.yaml'),cv2.FILE_STORAGE_READ)
    try:
        assert storage.isOpened()
        for name in ('cam0','cam1'):
            camera=storage.getNode(name)
            assert camera.getNode('resolution').at(0).real()==1280
            assert camera.getNode('intrinsics').at(0).real()==pytest.approx(762.7223968505859)
            assert camera.getNode('T_imu_cam').at(0).at(3).real()==pytest.approx(.0999100158967)
    finally:storage.release()


def test_missing_or_incompatible_source_pair_rejects_before_output(tmp_path):
    tree=ET.parse(SCENE);mono_sensors(tree,'sky_beta_estimated')[0].find('camera/image/width').text='1280'
    source=tmp_path/'wrong.sdf';tree.write(source)
    out=tmp_path/'no-output'
    with pytest.raises(ValueError):build(source,CONFIG,out)
    assert not out.exists()
    with pytest.raises(ValueError):build(SCENE,CONFIG,out,model_name='missing')


def test_course_variant_include_and_every_other_sensor_survive(tmp_path):
    tree=ET.parse(SCENE);tree.find('world/include/uri').text='model://dronehacks_course_alternate_surface'
    source=tmp_path/'variant.sdf';tree.write(source)
    out=tmp_path/'native';build(source,CONFIG,out)
    changed=ET.parse(out/'world.sdf')
    assert changed.findtext('world/include/uri')=='model://dronehacks_course_alternate_surface'
    for name in ('rgb','ideal_depth','down_range','imu_sensor','observer'):
        a=tree.findall(f".//sensor[@name='{name}']");b=changed.findall(f".//sensor[@name='{name}']")
        assert len(a)==len(b)
        for original,modified in zip(a,b):assert ET.tostring(original)==ET.tostring(modified)
