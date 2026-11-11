"""Offline stereo/wall/VIO experiment with recursively predicted pose priors.

Unlike fixed-fixture-neighborhood tests, matching priors here use only declared
launch initialization, recorded VIO increments and earlier wall observations.
Independent scene poses are loaded only AFTER the estimator loop for evaluation.
This is not a live estimator, calibrated covariance, or flight qualification.
"""
import argparse,json,sys
from dataclasses import asdict
from hashlib import sha256
from pathlib import Path
import numpy as np
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dronebench.depth_mapping import Pinhole
from dronebench.published_stereo import PublishedSparseStereo
from dronebench.published_raycast import build_published_raycast
from dronebench.published_wall_map import build_published_wall_map
from dronebench.depth_wall_constraint import extract_wall_constraints,WallConstraintConfig
from dronebench.wall_fusion import WallPositionFilter
from dronebench.wall_kinematic_filter import WallKinematicFilter
from dronebench.vision_odometry import _covariance
from tools.compare_vio_scene import matrix_wxyz,scene_pose


def main():
    import cv2
    cv2.setNumThreads(1)
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('evidence','repo','airframe-contract','truth-poses','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--launch-xy',type=float,nargs=2,required=True)
    parser.add_argument('--height',type=float,required=True)
    parser.add_argument('--height-error',type=float,required=True)
    parser.add_argument('--floor-z',type=float,required=True)
    parser.add_argument('--kinematic-vio',type=Path,help='full source-stamped VIO stream for position/velocity/bias filter')
    parser.add_argument('--wall-min-inliers',type=int,default=8)
    parser.add_argument('--wall-inlier-ratio',type=float,default=.7)
    args=parser.parse_args()
    wall_config=WallConstraintConfig(min_inliers=args.wall_min_inliers,min_inlier_ratio=args.wall_inlier_ratio)
    sys.path.insert(0,str(args.repo.resolve()))
    from drl.config import load_config
    from drl.course import load_obj_groups
    config,course,_=load_config(str(args.repo.resolve()/'config/course_v3.yaml'))
    obj=args.repo/config['course_obj']; _,groups=load_obj_groups(str(obj))
    net=np.concatenate([p for n,p in groups.items() if n.startswith('Net')])
    bounds=tuple(v for axis in range(3) for v in (net[:,axis].min(),net[:,axis].max()))
    ray_map=build_published_raycast(course,published_provenance='published OBJ:'+sha256(obj.read_bytes()).hexdigest(),
                                  net_bounds=bounds,floor_z=args.floor_z)
    walls=build_published_wall_map(course,z=args.height,net_bounds=bounds[:4])
    chain=yaml.safe_load((args.evidence/'calibration.yaml').read_text().replace('%YAML:1.0',''))
    fx,fy,cx,cy=chain['cam0']['intrinsics']; width,height=chain['cam0']['resolution']
    if not np.allclose(chain['cam0']['intrinsics'],chain['cam1']['intrinsics'],atol=1e-4) or any(abs(x)>1e-10 for x in chain['cam0']['distortion_coeffs']+chain['cam1']['distortion_coeffs']):
        raise ValueError('equal calibrated rectified intrinsics required')
    T=np.asarray(json.loads(args.airframe_contract.read_text())['body_from_imu'],float)
    sensor=PublishedSparseStereo(ray_map,Pinhole(width,height,fx,fy,cx,cy),
        T@np.asarray(chain['cam0']['T_imu_cam']),T@np.asarray(chain['cam1']['T_imu_cam']))
    pairs=json.loads((args.evidence/'pairs.json').read_text())['pairs']
    vio=json.loads((args.evidence/'vio_samples.json').read_text()); samples={r['left_stamp_ns']:r['observation'] for r in vio['samples']}
    def orientation(row):return matrix_wxyz([row['orientation'][k] for k in ('w','x','y','z')])
    north=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    align=north@T[:3,:3]@orientation(vio['first_observation']).T
    imu_origin=np.r_[args.launch_xy,args.height]+north@T[:3,3]
    args.output.mkdir(parents=True,exist_ok=False)
    state=None; previous=None; previous_t=None; records=[]
    full_vio=None; stream_index=0; filter_time=None; last_velocity_time=None
    def body_sample(row):
        if row['frame_id']!='global' or row['child_frame_id']!='imu':raise ValueError('VIO frame contract changed')
        imu_rotation=align@orientation(row); rotation=imu_rotation@T[:3,:3].T
        raw=imu_origin+align@np.array([row['position'][k] for k in ('x','y','z')])-rotation@T[:3,3]
        angular=T[:3,:3]@np.array([row['angular_velocity'][k] for k in ('x','y','z')])
        velocity=imu_rotation@np.array([row['linear_velocity'][k] for k in ('x','y','z')])-rotation@np.cross(angular,T[:3,3])
        x,y,z=T[:3,3]; skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
        J=np.c_[imu_rotation,rotation@skew@T[:3,:3]]
        covariance=J@_covariance(row['twist_covariance'],'recorded VIO twist covariance')@J.T
        arm=T[:3,:3].T@T[:3,3]; x,y,z=arm
        arm_skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
        pose_J=np.c_[align,imu_rotation@arm_skew]
        pose_covariance=pose_J@_covariance(row['pose_covariance'],'recorded VIO pose covariance')@pose_J.T
        return raw,rotation,velocity,covariance,pose_covariance
    if args.kinematic_vio:
        full_vio=[json.loads(line) for line in args.kinematic_vio.read_text().splitlines()]
        if not full_vio or np.any(np.diff([r['source_s'] for r in full_vio])<0):raise ValueError('advancing non-regressing VIO stream required')
        first,_,velocity,_,_=body_sample(full_vio[0])
        state=WallKinematicFilter(first[:2],velocity[:2]); filter_time=last_velocity_time=full_vio[0]['source_s']
    for pair in pairs:
        stamp=pair['left_stamp_ns']; t=stamp/1e9; row=samples[stamp]
        if abs(stamp-pair['right_stamp_ns'])>10_000_000 or abs(row['source_s']-t)>.05:
            raise ValueError('image/attitude source synchronization failed')
        for name in ('left','right'):
            if sha256((args.evidence/pair[name]).read_bytes()).hexdigest()!=pair[name+'_sha256']:raise ValueError('image hash mismatch')
        if full_vio is not None:
            latest=None
            while stream_index<len(full_vio) and full_vio[stream_index]['source_s']<=t:
                latest=full_vio[stream_index]; source=latest['source_s']; stream_index+=1
                if source<filter_time:continue
                if source>filter_time:state.predict(source-filter_time)
                filter_time=source
                if source-last_velocity_time>=.1:
                    position,_,velocity,covariance,pose_covariance=body_sample(latest)
                    state.observe_vio_position(position[:2],pose_covariance[:2,:2])
                    state.observe_velocity(velocity[:2],covariance[:2,:2]); last_velocity_time=source
            if latest is None or t-latest['source_s']>.05:raise ValueError('no causally fresh VIO sample at image source')
            row=latest
            if t>filter_time:state.predict(t-filter_time); filter_time=t
        raw,rotation,_,_,_=body_sample(row)
        if full_vio is None:
            if state is None:state=WallPositionFilter(raw[:2],np.eye(2)*.05**2)
            else:state.predict(raw[:2]-previous,t-previous_t)
        previous=raw[:2].copy(); previous_t=t
        images=[cv2.imread(str(args.evidence/pair[name]),cv2.IMREAD_GRAYSCALE) for name in ('left','right')]
        covariance_xy=state.covariance[:2,:2]
        error=np.r_[3*np.sqrt(np.diag(covariance_xy)),args.height_error]
        measured=sensor.observe(*images,position=np.r_[state.position,args.height],rotation=rotation,
                                error_xyz=error,angular_error_rad=.02)
        yaw=float(np.arctan2(rotation[1,0],rotation[0,0])); c,s=np.cos(yaw),np.sin(yaw)
        level=np.array([[c,s,0],[-s,c,0],[0,0,1.]])@rotation
        points=measured.points_base_xyz@level.T
        points=points[(points[:,2]+args.height>.3)&(points[:,2]+args.height<1.5)]
        factors=extract_wall_constraints(points[:,:2],walls.segments,(*state.position,yaw),config=wall_config)
        normals=[]
        for factor in factors.constraints:
            if factor.accepted:
                accepted=state.observe_wall(factor.normal,factor.projection,factor.variance) if full_vio is not None else state.update(factor.normal,factor.projection,factor.variance)
                if accepted:normals.append(factor.normal)
        eigen=np.linalg.eigvalsh(np.asarray(normals).T@np.asarray(normals)) if normals else np.zeros(2)
        rank=int(np.sum(eigen>max(eigen.max()*.01,1e-9)))
        records.append(dict(source_s=t,raw_xy_m=raw[:2].tolist(),filtered_xy_m=state.position.tolist(),
            covariance_xy_m2=state.covariance[:2,:2].tolist(),maximum_xy_std_m=float(np.sqrt(np.linalg.eigvalsh(state.covariance[:2,:2]).max())),
            wall_normal_rank=rank,factors_fused=len(normals),factors=[asdict(f) for f in factors.constraints],
            sensor=measured.diagnostics,matching_pose_bound_xyz_m=error.tolist()))
    truth=np.genfromtxt(args.truth_poses,delimiter=',',names=True)
    truth_rows=np.stack([truth[k] for k in ('t','x','y','z','qw','qx','qy','qz')],axis=1)
    for row in records:
        p,_=scene_pose(truth['t'],truth_rows,row['source_s'])
        row['raw_xy_error_m']=float(np.linalg.norm(np.array(row['raw_xy_m'])-p[:2]))
        row['filtered_xy_error_m']=float(np.linalg.norm(np.array(row['filtered_xy_m'])-p[:2]))
    report=dict(kind='offline_recursive_measured_stereo_wall_experiment_not_live_flight_estimator',records=records,
        frames=len(records),factors_fused=sum(r['factors_fused'] for r in records),
        raw_xy_rmse_m=float(np.sqrt(np.mean([r['raw_xy_error_m']**2 for r in records]))),
        filtered_xy_rmse_m=float(np.sqrt(np.mean([r['filtered_xy_error_m']**2 for r in records]))),
        filtered_max_xy_error_m=max(r['filtered_xy_error_m'] for r in records),
        median_sensor_wall_s=float(np.median([r['sensor']['wall_s'] for r in records])),
        rank_two_frames=sum(r['wall_normal_rank']==2 for r in records),flight_input_created=False,
        prior_scope='declared launch/static-init frame; recursive VIO observations and prior wall factors; no commanded fixture midpoint',
        estimator_model='position_velocity_with_position_and_velocity_biases' if full_vio is not None else 'position_increment_heuristic',
        wall_observation_config=asdict(wall_config),
        limitations='Uncalibrated uncertainty/cross-correlations, partial published geometry, fixed diagnostic height, sparse recorded timestamps; no live source-age/reset/authority qualification.')
    (args.output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2))


if __name__=='__main__':main()
