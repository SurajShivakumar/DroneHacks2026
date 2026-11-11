"""Offline rank-1 wall/VIO-increment experiment. No ROS pose publication."""
import argparse
from dataclasses import asdict
from hashlib import sha256
import json
from pathlib import Path
import sys
import numpy as np
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dronebench.depth_wall_constraint import extract_wall_constraints
from dronebench.published_wall_map import build_published_wall_map
from dronebench.wall_fusion import WallPositionFilter
from tools.compare_vio_scene import matrix_wxyz,scene_pose


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('depth-run','evidence','repo','airframe-contract','truth-poses','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--launch-xy',type=float,nargs=2,required=True)
    parser.add_argument('--height',type=float,required=True)
    args=parser.parse_args()
    sys.path.insert(0,str(args.repo.resolve()))
    from drl.config import load_config
    from drl.course import load_obj_groups
    config,course,_=load_config(str(args.repo.resolve()/'config/course_v3.yaml'))
    obj=args.repo/config['course_obj']; _,groups=load_obj_groups(str(obj))
    net=np.concatenate([p for name,p in groups.items() if name.startswith('Net')])
    bounds=(net[:,0].min(),net[:,0].max(),net[:,1].min(),net[:,1].max())
    walls=build_published_wall_map(course,z=args.height,net_bounds=bounds)
    sparse=(args.depth_run/'clouds.json').is_file()
    manifest_path=args.depth_run/'clouds.json' if sparse else args.depth_run/'stereo-depth-evidence/sequence.json'
    manifest=json.loads(manifest_path.read_text())
    if (sparse and manifest.get('kind')!='actual_sparse_stereo_with_declared_published_prior') or (
            not sparse and manifest['provenance']!='actual_stereo_published_map_pixel_priors_not_ideal_depth'):
        raise ValueError('actual published-prior stereo evidence required; no ideal-depth substitute')
    chain=yaml.safe_load((args.evidence/'calibration.yaml').read_text().replace('%YAML:1.0',''))
    fx,fy,cx,cy=chain['cam0']['intrinsics']
    body_from_imu=np.asarray(json.loads(args.airframe_contract.read_text())['body_from_imu'],float)
    body_from_camera=body_from_imu@np.asarray(chain['cam0']['T_imu_cam'])
    vio=json.loads((args.evidence/'vio_samples.json').read_text())
    samples={r['left_stamp_ns']:r['observation'] for r in vio['samples']}
    def orientation(row):return matrix_wxyz([row['orientation'][k] for k in ('w','x','y','z')])
    # Reviewed physical launch/orientation prior, NOT alignment to scene truth.
    north=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    align=north@body_from_imu[:3,:3]@orientation(vio['first_observation']).T
    imu_origin=np.r_[args.launch_xy,args.height]+north@body_from_imu[:3,3]
    state=None; previous=None; previous_t=None; records=[]
    for frame in manifest['frames']:
        row=samples[frame['stamp_ns']]; t=frame['stamp_ns']/1e9
        path=args.depth_run/frame['file'] if sparse else args.depth_run/'stereo-depth-evidence'/frame['file']
        if sha256(path.read_bytes()).hexdigest()!=frame['sha256']:raise ValueError('depth evidence hash mismatch')
        rotation=align@orientation(row)@body_from_imu[:3,:3].T
        raw=imu_origin+align@np.array([row['position'][k] for k in ('x','y','z')])-rotation@body_from_imu[:3,3]
        if state is None:state=WallPositionFilter(raw[:2],np.eye(2)*.05**2)
        else:state.predict(raw[:2]-previous,t-previous_t)
        previous=raw[:2]; previous_t=t
        yaw=float(np.arctan2(rotation[1,0],rotation[0,0])); c,s=np.cos(yaw),np.sin(yaw)
        level=np.array([[c,s,0],[-s,c,0],[0,0,1]])@rotation
        depth=np.load(path,allow_pickle=False)
        if sparse:
            if depth.ndim!=2 or depth.shape[1]!=3 or not np.isfinite(depth).all() or np.any(depth[:,2]<=0):
                raise ValueError('finite calibrated u,v,Z sparse points required')
            z=depth[:,2]; optical=np.c_[(depth[:,0]-cx)*z/fx,(depth[:,1]-cy)*z/fy,z]
        else:
            stride=manifest['pixel_stride']; v,u=np.indices(depth.shape)
            valid=np.isfinite(depth)&(depth>.25)&(depth<8); z=depth[valid]
            optical=np.c_[(u[valid]*stride-cx)*z/fx,(v[valid]*stride-cy)*z/fy,z]
        points=(optical@body_from_camera[:3,:3].T+body_from_camera[:3,3])@level.T
        points=points[(points[:,2]+args.height>.3)&(points[:,2]+args.height<1.5)]
        observations=extract_wall_constraints(points[:,:2],walls.segments,(*state.position,yaw))
        fused=0
        for factor in observations.constraints:
            if factor.accepted:
                fused+=state.update(factor.normal,factor.projection,factor.variance)
        records.append(dict(source_s=t,raw_vio_xy_m=raw[:2].tolist(),filtered_xy_m=state.position.tolist(),
            covariance_xy_m2=state.covariance.tolist(),factors_fused=int(fused),
            factors=[asdict(f) for f in observations.constraints]))
    # Independent evaluation happens after all state/factor calculations.
    truth=np.genfromtxt(args.truth_poses,delimiter=',',names=True)
    truth_rows=np.stack([truth[k] for k in ('t','x','y','z','qw','qx','qy','qz')],axis=1)
    raw_errors=[]; filtered_errors=[]
    for record in records:
        p,_=scene_pose(truth['t'],truth_rows,record['source_s'])
        a=float(np.linalg.norm(np.array(record['raw_vio_xy_m'])-p[:2])); b=float(np.linalg.norm(np.array(record['filtered_xy_m'])-p[:2]))
        record.update(raw_xy_error_m=a,filtered_xy_error_m=b); raw_errors.append(a); filtered_errors.append(b)
    result=dict(kind='offline_xy_wall_factor_experiment_not_flight_odometry_or_qualification',records=records,
        frames=len(records),factors_fused=sum(r['factors_fused'] for r in records),
        initialization='declared physical launch XY/height/north yaw and OpenVINS static initialization origin; no truth alignment',
        raw_xy_rmse_m=float(np.sqrt(np.mean(np.square(raw_errors)))),filtered_xy_rmse_m=float(np.sqrt(np.mean(np.square(filtered_errors)))),
        raw_max_xy_error_m=max(raw_errors),filtered_max_xy_error_m=max(filtered_errors),
        limitations='Heuristic process covariance and shared map/yaw prior correlations; no timestamp/freshness/reset/height/velocity contract or flight output.',
        flight_input_created=False,source_depth_manifest_sha256=sha256(manifest_path.read_bytes()).hexdigest())
    with args.output.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k!='records'},indent=2))


if __name__=='__main__':main()
