"""Offline diagnostic of ideal rendered depth against a published map.

No flight input/output. Uses a declared coarse fixture-area seed, virtual
calibration and recorded VIO attitude. Scene truth evaluates the result only.
This does not validate stereo depth or establish a hardware position fix.
"""
import argparse
from dataclasses import asdict
from hashlib import sha256
import json
from math import atan2,pi
from pathlib import Path
import sys
import numpy as np
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dronebench.depth_alignment import align_points_to_segments
from dronebench.published_wall_map import build_published_wall_map
from tools.compare_vio_scene import matrix_wxyz,scene_pose


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('run','repo','calibration','poses','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--height',type=float,required=True)
    args=parser.parse_args()
    sys.path.insert(0,str(args.repo.resolve()))
    from drl.config import load_config
    from drl.course import load_obj_groups
    config,course,_=load_config(str(args.repo.resolve()/'config/course_v3.yaml'))
    obj=args.repo/config['course_obj']
    _,groups=load_obj_groups(str(obj))
    net=np.concatenate([v for name,v in groups.items() if name.startswith('Net')])
    bounds=(float(net[:,0].min()),float(net[:,0].max()),float(net[:,1].min()),float(net[:,1].max()))
    walls=build_published_wall_map(course,z=args.height,net_bounds=bounds)
    manifest_path=args.run/'ideal-depth-evidence/sequence.json'
    manifest=json.loads(manifest_path.read_text())
    if manifest['provenance']!='rendered_ideal_depth_geometry_diagnostic_not_stereo_or_flight_input':
        raise ValueError('this tool is for the explicitly labeled ideal-depth diagnostic')
    calibration=yaml.safe_load(args.calibration.read_text().replace('%YAML:1.0',''))
    fx,fy,cx,cy=calibration['cam0']['intrinsics']
    transform=np.asarray(calibration['cam0']['T_imu_cam'],float)
    transform[:3,3]=(np.asarray(calibration['cam0']['T_imu_cam'])[:3,3]+
                        np.asarray(calibration['cam1']['T_imu_cam'])[:3,3])/2
    contract=json.loads((args.run/'settings/airframe-contract.json').read_text())
    body_from_depth=np.asarray(contract['body_from_imu'])@transform
    odometry=[json.loads(s) for s in (args.run/'vio-odometry.jsonl').read_text().splitlines()]
    stamps=np.array([r['source_s'] for r in odometry])
    def orientation(row):return matrix_wxyz([row['orientation'][k] for k in ('w','x','y','z')])
    initial=orientation(odometry[0]); initial_yaw=atan2(initial[1,0],initial[0,0])
    truth=np.genfromtxt(args.poses,delimiter=',',names=True)
    truth_times=truth['t']; truth_rows=np.stack([truth[k] for k in ('t','x','y','z','qw','qx','qy','qz')],axis=1)
    records=[]
    for frame in manifest['frames']:
        t=frame['stamp_ns']/1e9
        if not max(14.,stamps[0],truth_times[0])<=t<=min(75.,stamps[-1],truth_times[-1]):continue
        i=int(np.argmin(abs(stamps-t)))
        if abs(stamps[i]-t)>.05:continue
        path=args.run/'ideal-depth-evidence'/frame['file']
        if sha256(path.read_bytes()).hexdigest()!=frame['sha256']:raise ValueError('depth evidence hash mismatch')
        if frame['source_shape']!=list(calibration['cam0']['resolution'][::-1]):raise ValueError('depth calibration dimensions mismatch')
        depth=np.load(path,allow_pickle=False); stride=manifest['pixel_stride']
        v,u=np.indices(depth.shape); u=u*stride; v=v*stride
        valid=np.isfinite(depth)&(depth>.2)&(depth<5.)
        z=depth[valid]
        optical=np.c_[(u[valid]-cx)*z/fx,(v[valid]-cy)*z/fy,z]
        body=optical@body_from_depth[:3,:3].T+body_from_depth[:3,3]
        rotation=orientation(odometry[i]); yaw=atan2(rotation[1,0],rotation[0,0])
        c,s=np.cos(yaw),np.sin(yaw)
        level=np.array([[c,s,0],[-s,c,0],[0,0,1]])@rotation
        leveled=body@level.T
        leveled=leveled[(leveled[:,2]+args.height>.3)&(leveled[:,2]+args.height<1.5)]
        # Explicit coarse diagnostic prior: middle of the commanded rail area,
        # not a pose read from the simulator. VIO supplies attitude only.
        seed=(-2.515,.914+.4,pi/2+yaw-initial_yaw)
        result=align_points_to_segments(leveled[:,:2],walls.segments,seed)
        record=asdict(result); record['covariance']=result.covariance.tolist() if result.covariance is not None else None
        record.update(source_s=t,position_error_m=None)
        if result.accepted:
            p,_=scene_pose(truth_times,truth_rows,t)
            record['position_error_m']=float(np.linalg.norm(np.array(result.pose[:2])-p[:2]))
        records.append(record)
    accepted=[r['position_error_m'] for r in records if r['accepted']]
    report=dict(kind='offline_ideal_depth_geometry_only_not_stereo_or_flight_qualification',
        seed_scope='fixed midpoint of commanded diagnostic rail; physical initial yaw plus VIO attitude changes',
        calibration_scope='virtual common pinhole profile; centered depth sensor uses stereo-center extrinsics',
        source_obj_sha256=sha256(obj.read_bytes()).hexdigest(),depth_manifest_sha256=sha256(manifest_path.read_bytes()).hexdigest(),
        map_scope=walls.source_scope,records=records,accepted=len(accepted),attempted=len(records),
        position_rmse_m=float(np.sqrt(np.mean(np.square(accepted)))) if accepted else None,
        maximum_position_error_m=max(accepted) if accepted else None,flight_input_created=False)
    with args.output.open('x',encoding='utf-8') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2))


if __name__=='__main__':main()
