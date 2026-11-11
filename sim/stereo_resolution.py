"""Offline 640x400 -> 1280x800 mono-stereo sensitivity variant.

Only left/right render dimensions and their pinhole intrinsics/resolution
change. This does not qualify Gazebo runtime, estimator health or hardware.
"""
import argparse
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import yaml


def digest(path):return sha256(path.read_bytes()).hexdigest()


def load_yaml(path):
    return yaml.safe_load(path.read_text(encoding='utf-8-sig').replace('%YAML:1.0',''))


def relative_dependency(config,key):
    name=config[key]
    if not isinstance(name,str) or Path(name).name!=name or '/' in name or '\\' in name or name in ('.','..'):
        raise ValueError('reviewed sibling calibration filenames required')
    return name


def canonical_yaml(value):return yaml.safe_dump(value,sort_keys=True)


def opencv_camera_yaml(chain):
    """Keep nested arrays in flow style for OpenVINS' OpenCV FileStorage.

    PyYAML's indentationless block sequences are valid YAML but rejected by
    the pinned OpenCV loader. This matches the existing Kalibr input format.
    """
    lines=['%YAML:1.0']
    for camera,fields in chain.items():
        lines.append(camera+':')
        for key,value in fields.items():
            lines.append('  '+key+': '+json.dumps(value,allow_nan=False))
    return '\n'.join(lines)+'\n'


def mono_sensors(tree,model_name):
    models=tree.findall(f".//model[@name='{model_name}']")
    if len(models)!=1:raise ValueError('one explicitly selected aircraft model required')
    pair=[]
    for name in ('left','right'):
        sensors=models[0].findall(f".//sensor[@name='{name}']")
        if len(sensors)!=1 or sensors[0].get('type')!='camera':raise ValueError('one left/right mono camera pair required')
        sensor=sensors[0]
        if sensor.findtext('camera/image/format')!='L8':raise ValueError('requires existing L8 stereo render inputs')
        if sensor.find('camera/intrinsics') is not None or sensor.find('camera/lens') is not None:
            raise ValueError('explicit SDF intrinsics/lens need separately reviewed scaling')
        pair.append(sensor)
    return pair


def build(scene,vio_config,output,*,model_name='sky_beta_estimated',guard_source=None):
    scene=Path(scene);vio_config=Path(vio_config);output=Path(output)
    if output.exists():raise FileExistsError('preserve existing resolution evidence')
    if not isinstance(model_name,str) or not model_name or any(c in model_name for c in "'\"[]"):
        raise ValueError('simple explicit aircraft model name required')
    config=load_yaml(vio_config)
    if config.get('use_stereo') is not True or config.get('max_cameras')!=2 or config.get('downsample_cameras') is not False:
        raise ValueError('unchanged stereo two-camera config without downsampling required')
    chain_name=relative_dependency(config,'relative_config_imucam');imu_name=relative_dependency(config,'relative_config_imu')
    if len({chain_name,imu_name,vio_config.name})!=3:raise ValueError('distinct config/calibration filenames required')
    chain_path=vio_config.parent/chain_name;imu_path=vio_config.parent/imu_name
    chain=load_yaml(chain_path)
    if set(chain)!=set(('cam0','cam1')):raise ValueError('exact stereo calibration required')
    tree=ET.parse(scene);source_xml=ET.tostring(tree.getroot());before=deepcopy(tree)
    pair=mono_sensors(tree,model_name)
    calibrations=deepcopy(chain);dimensions=[]
    for name,sensor in zip(('cam0','cam1'),pair):
        cam=calibrations[name]
        w=int(sensor.findtext('camera/image/width'));h=int(sensor.findtext('camera/image/height'))
        hfov=float(sensor.findtext('camera/horizontal_fov'))
        intrinsics=np.asarray(cam['intrinsics'],float)
        if ([w,h]!=[640,400] or cam['resolution']!=[w,h] or intrinsics.shape!=(4,)
                or not np.isfinite([*intrinsics,hfov]).all() or min(intrinsics[:2])<=0 or not 0<hfov<np.pi
                or cam.get('camera_model')!='pinhole' or cam.get('distortion_model')!='radtan'
                or any(abs(float(x))>1e-10 for x in cam['distortion_coeffs'])):
            raise ValueError('source must be calibrated rectified 640x400 pinhole mono pair')
        expected_fx=w/(2*np.tan(hfov/2))
        if not np.isclose(intrinsics[0],expected_fx,rtol=1e-6) or not np.allclose(intrinsics[2:],[w/2,h/2],atol=1e-7):
            raise ValueError('source pinhole intrinsics do not match FOV/centred image')
        for axis,value in (('width',w*2),('height',h*2)):sensor.find('camera/image/'+axis).text=str(value)
        cam['intrinsics']=(intrinsics*2).tolist();cam['resolution']=[w*2,h*2]
        dimensions.append(dict(camera=name,source_resolution=[w,h],variant_resolution=[w*2,h*2],
            source_intrinsics=intrinsics.tolist(),variant_intrinsics=cam['intrinsics'],horizontal_fov_rad=hfov))
    transforms=[np.asarray(chain[name]['T_imu_cam'],float) for name in ('cam0','cam1')]
    for T in transforms:
        if (T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1])
                or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(T[:3,:3]),1)):
            raise ValueError('proper calibrated stereo extrinsics required')
    baseline=transforms[0][:3,:3].T@(transforms[1][:3,3]-transforms[0][:3,3])
    if not np.allclose(transforms[0][:3,:3],transforms[1][:3,:3]) or baseline[0]<=0 or not np.allclose(baseline[1:],0,atol=1e-8):
        raise ValueError('positive rectified optical-X stereo baseline required')
    # Revert exactly the four permitted XML text fields; everything else must
    # equal the parsed source, including all geometry, physics and other sensors.
    reverted=deepcopy(tree)
    for sensor in mono_sensors(reverted,model_name):
        sensor.find('camera/image/width').text='640';sensor.find('camera/image/height').text='400'
    source_pair=mono_sensors(before,model_name)
    for a,b in zip(mono_sensors(reverted,model_name),source_pair):
        for axis in ('width','height'):a.find('camera/image/'+axis).text=b.findtext('camera/image/'+axis)
    if ET.tostring(reverted.getroot())!=source_xml:raise RuntimeError('resolution variant changed unrelated SDF content')
    reverted_chain=deepcopy(calibrations)
    for name in ('cam0','cam1'):
        for key in ('intrinsics','resolution'):reverted_chain[name][key]=chain[name][key]
    if canonical_yaml(reverted_chain)!=canonical_yaml(chain):raise RuntimeError('variant changed unrelated camera calibration')
    # Fail on absent dependencies before creating output evidence.
    imu_bytes=imu_path.read_bytes();config_bytes=vio_config.read_bytes()
    if guard_source is None:guard_source=Path(__file__).resolve().parents[1]/'dronebench/vio_quality.py'
    guard_source=Path(guard_source);guard_bytes=guard_source.read_bytes()
    output.mkdir(parents=True,exist_ok=False);(output/'vio').mkdir();(output/'source').mkdir()
    tree.write(output/'world.sdf',encoding='utf-8',xml_declaration=True)
    (output/'vio'/vio_config.name).write_bytes(config_bytes)
    (output/'vio'/imu_name).write_bytes(imu_bytes)
    (output/'vio'/chain_name).write_text(opencv_camera_yaml(calibrations),encoding='utf-8')
    for path,name in ((scene,'world.sdf'),(vio_config,vio_config.name),(chain_path,chain_name),(imu_path,imu_name),(guard_source,'vio_quality.py')):
        shutil.copy2(path,output/'source'/name)
    shutil.copy2(Path(__file__),output/'generator.py')
    if (load_yaml(output/'vio'/vio_config.name)!=config or digest(output/'vio'/vio_config.name)!=digest(vio_config)
            or (output/'vio'/imu_name).read_bytes()!=imu_bytes):raise RuntimeError('estimator knobs or IMU calibration changed')
    # Validate serialized YAML, not only the in-memory construction.
    if canonical_yaml(load_yaml(output/'vio'/chain_name))!=canonical_yaml(calibrations):raise RuntimeError('serialized calibration mismatch')
    report=dict(kind='offline_stereo_resolution_sensitivity_not_runtime_or_hardware_qualification',
        model_name=model_name,source_sha256={str(p):digest(p) for p in (scene,vio_config,chain_path,imu_path,guard_source)},
        generator_sha256=digest(Path(__file__)),variant_sha256={str(p.relative_to(output)):digest(p) for p in
            (output/'world.sdf',output/'vio'/vio_config.name,output/'vio'/chain_name,output/'vio'/imu_name)},
        camera_changes=dimensions,stereo_baseline_optical_m=baseline.tolist(),
        source_canonical_sdf_sha256=sha256(source_xml).hexdigest(),
        reverted_variant_canonical_sdf_sha256=sha256(ET.tostring(reverted.getroot())).hexdigest(),
        only_four_sdf_dimension_texts_changed=True,only_camera_intrinsics_and_resolution_changed=True,
        estimator_config_byte_identical=True,imu_calibration_byte_identical=True,guard_snapshot_sha256=sha256(guard_bytes).hexdigest(),
        feature_budget_and_all_estimator_knobs_unchanged=True,render_runtime_validated=False,flight_ready=False,
        hardware_references=['https://docs.luxonis.com/hardware/platform/sensors/focus-type',
                             'https://docs.luxonis.com/hardware/products/OAK-D%20S2'],
        limitations='Resolution-only sensitivity variant.640x400 remains the lighter G4 preset;1280x800 matches the documented mono active pixels, not hardware qualification. Four times rendered pixels may slow real-time factor, callback delivery and wall-time-driven fixture profiles. Keep source timestamps and motion evidence separate from wall time. FOV, extrinsics, baseline, all other sensors, masses, collisions, plugins, estimator settings and feature-guard source are unchanged.')
    (output/'resolution-contract.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('scene','vio-config','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--model-name',default='sky_beta_estimated')
    args=parser.parse_args();report=build(args.scene,args.vio_config,args.output,model_name=args.model_name)
    print(json.dumps(report,indent=2))
