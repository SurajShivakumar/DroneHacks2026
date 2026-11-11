"""Offline per-pixel published-map prior experiment; never a flight input."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys,time,os,platform
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dronebench.published_raycast import build_published_raycast
from dronebench.map_guided_stereo import MapGuidedStereoConfig,map_guided_stereo_depth
from dronebench.depth_recording import DepthSequence
from tools.review_stereo_pairs import reproject_center_depth


def depth_intervals(ray_map, optical_rays, body_from_camera, guidance, yaw, height, height_error):
    c,s=np.cos(yaw),np.sin(yaw); R=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    seed=np.r_[guidance['seed_xy_m'],height]
    offset=R@body_from_camera[:3,3]
    directions=optical_rays@(R@body_from_camera[:3,:3]).T
    hits=ray_map.cast(seed+offset,directions)
    if hits.origin_inside_solid:
        raise ValueError('declared pose prior camera starts inside published solid')
    denominator=np.abs(np.einsum('ni,ni->n',hits.normals,directions))
    translation=np.abs(hits.normals)@np.r_[guidance['xy_error_bounds_m'],height_error]
    rotation=guidance['yaw_error_bound_rad']*(hits.t*np.linalg.norm(directions,axis=1)+np.linalg.norm(offset))
    with np.errstate(divide='ignore',invalid='ignore'):error=(translation+rotation+.01)/denominator
    lo=np.maximum(.25,hits.t-error); hi=np.minimum(8.,hits.t+error)
    valid=np.isfinite(hits.t)&(denominator>.1)&(lo<hi)
    lo[~valid]=np.nan; hi[~valid]=np.nan
    return lo,hi


def main():
    import cv2,yaml
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('evidence','repo','airframe-contract','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--height',type=float,required=True)
    parser.add_argument('--height-error',type=float,required=True)
    parser.add_argument('--floor-z',type=float,required=True)
    parser.add_argument('--opencv-threads',type=int)
    parser.add_argument('--blas-threads',type=int)
    parser.add_argument('--limit',type=int,help='bounded profiling subset; report retains actual evaluated frame count')
    args=parser.parse_args()
    for value in (args.opencv_threads,args.blas_threads,args.limit):
        if value is not None and value<1:raise ValueError('positive thread/frame limits required')
    if args.opencv_threads is not None:cv2.setNumThreads(args.opencv_threads)
    thread_control=None
    if args.blas_threads is not None:
        from threadpoolctl import threadpool_limits
        thread_control=threadpool_limits(limits=args.blas_threads)
    if not np.isfinite([args.height,args.height_error,args.floor_z]).all() or args.height_error<=0:
        raise ValueError('finite declared height and positive height uncertainty required')
    sys.path.insert(0,str(args.repo.resolve()))
    from drl.config import load_config
    from drl.course import load_obj_groups
    config,course,_=load_config(str(args.repo.resolve()/'config/course_v3.yaml'))
    obj=args.repo/config['course_obj']; _,groups=load_obj_groups(str(obj))
    net=np.concatenate([points for name,points in groups.items() if name.startswith('Net')])
    bounds=(net[:,0].min(),net[:,0].max(),net[:,1].min(),net[:,1].max(),net[:,2].min(),net[:,2].max())
    map_model=build_published_raycast(course,published_provenance='published OBJ sha256:'+sha256(obj.read_bytes()).hexdigest(),
                                    net_bounds=bounds,floor_z=args.floor_z)
    manifest=json.loads((args.evidence/'pairs.json').read_text())
    guidance=json.loads((args.evidence/'guidance.json').read_text())
    attitudes={row['left_stamp_ns']:row['relative_yaw_rad'] for row in guidance['attitudes']}
    chain=yaml.safe_load((args.evidence/'calibration.yaml').read_text().replace('%YAML:1.0',''))
    camera=chain['cam0']; fx,fy,cx,cy=camera['intrinsics']; width,height=camera['resolution']
    left=np.asarray(camera['T_imu_cam'],float); right=np.asarray(chain['cam1']['T_imu_cam'],float)
    baseline=left[:3,:3].T@(right[:3,3]-left[:3,3])
    if not np.allclose(baseline[1:],0,atol=1e-7) or baseline[0]<=0 or not np.allclose(left[:3,:3],right[:3,:3],atol=1e-7):
        raise ValueError('rectified stereo extrinsics required')
    if not np.allclose(camera['intrinsics'],chain['cam1']['intrinsics'],atol=1e-4):raise ValueError('equal rectified intrinsics required')
    body_from_imu=np.asarray(json.loads(args.airframe_contract.read_text())['body_from_imu'])
    body_cameras=[body_from_imu@left,body_from_imu@right]
    center=left.copy(); center[:3,3]=(left[:3,3]+right[:3,3])/2
    left_from_center=np.linalg.inv(left)@center
    v,u=np.indices((height,width)); rays=np.c_[((u-cx)/fx).ravel(),((v-cy)/fy).ravel(),np.ones(width*height)]
    cfg=MapGuidedStereoConfig(fx=fx,baseline_m=float(baseline[0]))
    args.output.mkdir(parents=True,exist_ok=False)
    sequence=DepthSequence(args.output/'stereo-depth-evidence',topic='/offline/pixel_prior_stereo',
                           provenance='actual_stereo_published_map_pixel_priors_not_ideal_depth',period_ns=1)
    records=[]
    for pair in manifest['pairs'][:args.limit]:
        if abs(pair['left_stamp_ns']-pair['right_stamp_ns'])>10_000_000:raise ValueError('stereo timestamp skew')
        for name in ('left','right','ideal'):
            if sha256((args.evidence/pair[name]).read_bytes()).hexdigest()!=pair[name+'_sha256']:raise ValueError('input hash mismatch')
        images=[cv2.imread(str(args.evidence/pair[name]),cv2.IMREAD_GRAYSCALE) for name in ('left','right')]
        if any(image is None or image.shape!=(height,width) for image in images):raise ValueError('image calibration shape mismatch')
        yaw=guidance['initial_yaw_rad']+attitudes[pair['left_stamp_ns']]
        started=time.perf_counter(); intervals=[]
        for transform in body_cameras:
            lo,hi=depth_intervals(map_model,rays,transform,guidance,yaw,args.height,args.height_error)
            intervals.extend([lo.reshape(height,width),hi.reshape(height,width)])
        prior_elapsed=time.perf_counter()-started
        matching_started=time.perf_counter()
        result=map_guided_stereo_depth(*images,*intervals,config=cfg)
        matching_elapsed=time.perf_counter()-matching_started
        elapsed=time.perf_counter()-started
        # Only now load independent ideal-depth evaluation evidence.
        ideal=np.load(args.evidence/pair['ideal'],allow_pickle=False)
        ref=reproject_center_depth(ideal,pair['ideal_stride'],camera['intrinsics'],left_from_center,(height,width))
        common=np.isfinite(ref)&result.valid; error=np.abs(result.depth_m[common]-ref[common])
        sequence.append(result.depth_m,pair['left_stamp_ns'],'left_optical')
        records.append(dict(source_s=pair['left_stamp_ns']/1e9,wall_s=elapsed,
            prior_wall_s=prior_elapsed,matching_wall_s=matching_elapsed,matching=result.diagnostics,
            diagnostic_reference_pixels=len(error),median_absolute_depth_error_m=float(np.median(error)) if len(error) else None,
            p95_absolute_depth_error_m=float(np.percentile(error,95)) if len(error) else None))
    sequence.close()
    report=dict(kind='offline_declared_fixture_prior_stereo_experiment_not_pose_or_flight_qualification',
        records=records,frames=len(records),median_wall_s=float(np.median([r['wall_s'] for r in records])),
        ideal_depth_used_by_matcher=False,published_map_scope=map_model.source_scope,omitted_geometry=map_model.omitted_geometry,
        prior_scope=guidance['scope'],height_m=args.height,height_error_bound_m=args.height_error,
        runtime=dict(platform=platform.system(),architecture=platform.machine(),numpy=np.__version__,
                     opencv=cv2.__version__,openblas_num_threads=os.environ.get('OPENBLAS_NUM_THREADS'),
                     opencv_num_threads=cv2.getNumThreads(),requested_blas_threads=args.blas_threads),
        source_obj_sha256=sha256(obj.read_bytes()).hexdigest(),flight_input_created=False)
    (args.output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2))
    if thread_control is not None:thread_control.restore_original_limits()


if __name__=='__main__':main()
