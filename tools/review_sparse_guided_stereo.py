"""Offline sparse real-stereo experiment with declared published-map priors."""
import argparse,json,sys,time
from hashlib import sha256
from pathlib import Path
import numpy as np
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dronebench.map_guided_stereo import MapGuidedStereoConfig
from dronebench.sparse_map_stereo import match_sparse_patches
from dronebench.published_raycast import build_published_raycast
from tools.review_pixel_guided_stereo import depth_intervals
from tools.review_stereo_pairs import reproject_center_depth


def main():
    import cv2
    cv2.setNumThreads(1)
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('evidence','repo','airframe-contract','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--height',type=float,required=True)
    parser.add_argument('--height-error',type=float,required=True)
    parser.add_argument('--floor-z',type=float,required=True)
    parser.add_argument('--max-features',type=int,default=500)
    parser.add_argument('--feature-band-fraction',type=float,default=1.,
                        help='central horizontal image band for independent horizontal wall support')
    args=parser.parse_args()
    if not 20<=args.max_features<=1000:raise ValueError('20..1000 diagnostic features required')
    if not .1<=args.feature_band_fraction<=1.:raise ValueError('feature band must be 0.1..1.0 of image height')
    sys.path.insert(0,str(args.repo.resolve()))
    from drl.config import load_config
    from drl.course import load_obj_groups
    config,course,_=load_config(str(args.repo.resolve()/'config/course_v3.yaml'))
    obj=args.repo/config['course_obj']; _,groups=load_obj_groups(str(obj))
    net=np.concatenate([p for n,p in groups.items() if n.startswith('Net')])
    bounds=tuple(v for axis in range(3) for v in (net[:,axis].min(),net[:,axis].max()))
    ray_map=build_published_raycast(course,published_provenance='published OBJ:'+sha256(obj.read_bytes()).hexdigest(),
                                  net_bounds=bounds,floor_z=args.floor_z)
    pairs=json.loads((args.evidence/'pairs.json').read_text())['pairs']
    guidance=json.loads((args.evidence/'guidance.json').read_text())
    attitudes={r['left_stamp_ns']:r['relative_yaw_rad'] for r in guidance['attitudes']}
    chain=yaml.safe_load((args.evidence/'calibration.yaml').read_text().replace('%YAML:1.0',''))
    camera=chain['cam0']; fx,fy,cx,cy=camera['intrinsics']; width,height=camera['resolution']
    left=np.asarray(camera['T_imu_cam'],float); right=np.asarray(chain['cam1']['T_imu_cam'],float)
    baseline=left[:3,:3].T@(right[:3,3]-left[:3,3])
    if baseline[0]<=0 or not np.allclose(baseline[1:],0,atol=1e-7) or not np.allclose(left[:3,:3],right[:3,:3]):
        raise ValueError('parallel rectified stereo required')
    body_from_imu=np.asarray(json.loads(args.airframe_contract.read_text())['body_from_imu'])
    transforms=[body_from_imu@left,body_from_imu@right]
    center=left.copy(); center[:3,3]=(left[:3,3]+right[:3,3])/2; left_from_center=np.linalg.inv(left)@center
    cfg=MapGuidedStereoConfig(fx=fx,baseline_m=float(baseline[0]))
    args.output.mkdir(parents=True,exist_ok=False); records=[]; frames=[]
    def rays(uv):return np.c_[(uv[:,0]-cx)/fx,(uv[:,1]-cy)/fy,np.ones(len(uv))]
    for index,pair in enumerate(pairs):
        if abs(pair['left_stamp_ns']-pair['right_stamp_ns'])>10_000_000:raise ValueError('stereo clock skew')
        for name in ('left','right','ideal'):
            if sha256((args.evidence/pair[name]).read_bytes()).hexdigest()!=pair[name+'_sha256']:raise ValueError('input hash mismatch')
        images=[cv2.imread(str(args.evidence/pair[name]),cv2.IMREAD_GRAYSCALE) for name in ('left','right')]
        if any(image is None or image.shape!=(height,width) for image in images):raise ValueError('calibration dimensions mismatch')
        started=time.perf_counter()
        mask=np.zeros(images[0].shape,np.uint8)
        top=int(height*(1-args.feature_band_fraction)/2); bottom=height-top
        mask[top:bottom,:]=255
        features=cv2.goodFeaturesToTrack(images[0],maxCorners=args.max_features,qualityLevel=.01,minDistance=8,blockSize=5,mask=mask)
        uv=features.reshape(-1,2) if features is not None else np.empty((0,2))
        yaw=guidance['initial_yaw_rad']+attitudes[pair['left_stamp_ns']]
        lo,hi=depth_intervals(ray_map,rays(uv),transforms[0],guidance,yaw,args.height,args.height_error)
        forward=match_sparse_patches(*images,uv,lo,hi,sign=1,config=cfg)
        selected=np.flatnonzero(forward.valid); reverse_uv=uv[selected].copy()
        reverse_uv[:,0]-=forward.disparity_px[selected]
        lo_r,hi_r=depth_intervals(ray_map,rays(reverse_uv),transforms[1],guidance,yaw,args.height,args.height_error)
        backward=match_sparse_patches(images[1],images[0],reverse_uv,lo_r,hi_r,sign=-1,config=cfg)
        good=backward.valid&(np.abs(forward.disparity_px[selected]+backward.disparity_px)<=cfg.lr_max_error_px)
        valid_indices=selected[good]; result_uvz=np.c_[uv[valid_indices],forward.depth_m[valid_indices]].astype(np.float32)
        elapsed=time.perf_counter()-started
        path=args.output/f'cloud_{index:06d}.npy'
        with path.open('xb') as stream:np.save(stream,result_uvz,allow_pickle=False)
        frames.append(dict(file=path.name,stamp_ns=pair['left_stamp_ns'],sha256=sha256(path.read_bytes()).hexdigest()))
        # Reference is evaluation ONLY, after matching and saved cloud output.
        ideal=np.load(args.evidence/pair['ideal'],allow_pickle=False)
        ref=reproject_center_depth(ideal,pair['ideal_stride'],camera['intrinsics'],left_from_center,(height,width))
        vv,uu=np.where(np.isfinite(ref)); errors=[]
        if len(uu) and len(result_uvz):
            distance=np.sum((result_uvz[:,None,:2]-np.c_[uu,vv][None,:,:])**2,axis=2)
            nearest=np.argmin(distance,axis=1); close=distance[np.arange(len(nearest)),nearest]<=16
            errors=np.abs(result_uvz[close,2]-ref[vv[nearest[close]],uu[nearest[close]]])
        records.append(dict(source_s=pair['left_stamp_ns']/1e9,wall_s=elapsed,keypoints=len(uv),valid_points=len(result_uvz),
            forward=forward.diagnostics,backward=backward.diagnostics,reference_points=len(errors),
            approximate_median_depth_error_m=float(np.median(errors)) if len(errors) else None,
            approximate_p95_depth_error_m=float(np.percentile(errors,95)) if len(errors) else None))
    (args.output/'clouds.json').write_text(json.dumps(dict(kind='actual_sparse_stereo_with_declared_published_prior',
        coordinate_convention='left optical u,v,Z in original calibrated pixels/metres',frames=frames),indent=2))
    report=dict(kind='offline_sparse_declared_prior_stereo_not_flight_qualification',records=records,
        frames=len(records),median_wall_s=float(np.median([r['wall_s'] for r in records])),
        median_valid_points=float(np.median([r['valid_points'] for r in records])),
        feature_band_fraction=args.feature_band_fraction,
        reference_scope='nearest sparse reprojected ideal sample <=4 pixels; occlusion/sampling errors possible',
        ideal_depth_used_by_matcher=False,flight_input_created=False,published_map_omissions=ray_map.omitted_geometry)
    (args.output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2))


if __name__=='__main__':main()
