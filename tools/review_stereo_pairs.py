"""Compute depth from actual recorded stereo; ideal depth is evaluation only."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
from dataclasses import replace
import sys
import time
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dronebench.stereo_depth import StereoDepthConfig,rectified_stereo_depth
from dronebench.depth_recording import DepthSequence


def net_guided_config(base, guidance, relative_yaw, camera_in_body):
    """Declared fixture-neighborhood prior, not measured/world pose access.

    Uses the central optical ray toward the published net. Other occluding
    surfaces are not modeled here; this remains a bounded offline experiment.
    """
    xy=np.asarray(guidance['seed_xy_m'],float)
    bounds=np.asarray(guidance['xy_error_bounds_m'],float)
    if (xy.shape!=(2,) or bounds.shape!=(2,) or np.any(bounds<0)
            or not np.isfinite([*xy,*bounds,*guidance['net_bounds_m'],relative_yaw,
                                guidance['initial_yaw_rad'],guidance['yaw_error_bound_rad']]).all()
            or guidance['yaw_error_bound_rad']<0):
        raise ValueError('finite declared pose/error prior required')
    yaw=guidance['initial_yaw_rad']+relative_yaw
    c,s=np.cos(yaw),np.sin(yaw); R=np.array([[c,-s],[s,c]])
    camera=xy+R@np.asarray(camera_in_body)[:2]; ray=R@np.array([1.,0.])
    xmin,xmax,ymin,ymax=guidance['net_bounds_m']; hits=[]
    for axis,limits in ((0,(xmin,xmax)),(1,(ymin,ymax))):
        if abs(ray[axis])<1e-8:continue
        for edge in limits:
            t=(edge-camera[axis])/ray[axis]; hit=camera+t*ray
            if t>0 and xmin-1e-8<=hit[0]<=xmax+1e-8 and ymin-1e-8<=hit[1]<=ymax+1e-8:
                hits.append((t,axis))
    if not hits:raise ValueError('declared net prior has no forward intersection')
    t,axis=min(hits)
    error=(bounds[axis]+.01+guidance['yaw_error_bound_rad']*(t+np.linalg.norm(camera_in_body)))/abs(ray[axis])
    lo=max(base.min_depth_m,t-error); hi=min(base.max_depth_m,t+error)
    if not 0<lo<hi:raise ValueError('empty declared prior depth interval')
    fb=base.fx*base.baseline_m; lower=int(np.floor(fb/hi)); upper=int(np.ceil(fb/lo))
    disparities=16*int(np.ceil((upper-lower+1)/16))
    return replace(base,min_disparity=lower,num_disparities=disparities,
                   min_depth_m=lo,max_depth_m=hi)


def reproject_center_depth(depth, stride, intrinsics, left_from_center, shape):
    """Sparse ideal-center diagnostic reference in the LEFT optical viewpoint."""
    fx,fy,cx,cy=intrinsics
    v,u=np.indices(depth.shape); good=np.isfinite(depth)&(depth>0)
    z=depth[good]
    points=np.c_[(u[good]*stride-cx)*z/fx,(v[good]*stride-cy)*z/fy,z]
    points=points@left_from_center[:3,:3].T+left_from_center[:3,3]
    valid=np.isfinite(points).all(axis=1)&(points[:,2]>0); points=points[valid]
    uu=np.rint(points[:,0]*fx/points[:,2]+cx).astype(int)
    vv=np.rint(points[:,1]*fy/points[:,2]+cy).astype(int)
    inside=(uu>=0)&(uu<shape[1])&(vv>=0)&(vv<shape[0])
    reference=np.full(shape,np.inf,dtype=float)
    np.minimum.at(reference,(vv[inside],uu[inside]),points[inside,2])
    reference[~np.isfinite(reference)]=np.nan
    return reference


def main():
    import cv2,yaml
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--guidance',type=Path,help='explicit coarse fixture-area prior; not an estimated pose fix')
    parser.add_argument('--airframe-contract',type=Path,help='calibrated body_from_imu required for guidance')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((args.evidence/'pairs.json').read_text())
    calibration=yaml.safe_load((args.evidence/'calibration.yaml').read_text().replace('%YAML:1.0',''))
    a,b=calibration['cam0'],calibration['cam1']
    left=np.asarray(a['T_imu_cam'],float); right=np.asarray(b['T_imu_cam'],float)
    relative=left[:3,:3].T@(right[:3,3]-left[:3,3])
    if (not np.allclose(left[:3,:3],right[:3,:3],atol=1e-7)
            or not np.allclose(a['intrinsics'],b['intrinsics'],atol=1e-4)
            or not np.allclose(relative[1:],0,atol=1e-7) or relative[0]<=0
            or any(abs(v)>1e-10 for v in a['distortion_coeffs']+b['distortion_coeffs'])):
        raise ValueError('parallel rectified equal-intrinsic stereo calibration required')
    center=left.copy(); center[:3,3]=(left[:3,3]+right[:3,3])/2
    left_from_center=np.linalg.inv(left)@center
    cfg=StereoDepthConfig(fx=a['intrinsics'][0],baseline_m=float(relative[0]))
    guidance=json.loads(args.guidance.read_text()) if args.guidance else None
    if guidance and not args.airframe_contract:
        raise ValueError('guided mode requires body/IMU lever-arm calibration')
    camera_in_body=(np.asarray(json.loads(args.airframe_contract.read_text())['body_from_imu'])@left)[:3,3] if guidance else None
    attitudes={r['left_stamp_ns']:r['relative_yaw_rad'] for r in guidance['attitudes']} if guidance else {}
    sequence=DepthSequence(args.output/'stereo-depth-evidence',topic='/offline/rectified_stereo_depth',
        provenance='actual_recorded_rectified_stereo_lr_checked_not_ideal_depth',period_ns=1)
    records=[]
    for pair in manifest['pairs']:
        if abs(pair['left_stamp_ns']-pair['right_stamp_ns'])>10_000_000:
            raise ValueError('unsynchronized stereo pair')
        for name in ('left','right','ideal'):
            if sha256((args.evidence/pair[name]).read_bytes()).hexdigest()!=pair[name+'_sha256']:
                raise ValueError('input image/depth hash mismatch')
        images=[cv2.imread(str(args.evidence/pair[name]),cv2.IMREAD_GRAYSCALE) for name in ('left','right')]
        if any(image is None or image.shape!=tuple(a['resolution'][::-1]) for image in images):
            raise ValueError('decoded stereo shape/calibration mismatch')
        selected_cfg=net_guided_config(cfg,guidance,attitudes[pair['left_stamp_ns']],camera_in_body) if guidance else cfg
        started=time.perf_counter(); result=rectified_stereo_depth(*images,selected_cfg); elapsed=time.perf_counter()-started
        # The estimator above has finished before evaluation loads ideal depth.
        ideal=np.load(args.evidence/pair['ideal'],allow_pickle=False)
        reference=reproject_center_depth(ideal,pair['ideal_stride'],a['intrinsics'],left_from_center,result.depth_m.shape)
        common=np.isfinite(reference)&result.valid
        errors=np.abs(result.depth_m[common]-reference[common])
        sequence.append(result.depth_m,pair['left_stamp_ns'],'left_optical')
        records.append(dict(source_s=pair['left_stamp_ns']/1e9,wall_s=elapsed,
            matching=result.diagnostics,diagnostic_reference_pixels=len(errors),
            median_absolute_depth_error_m=float(np.median(errors)) if len(errors) else None,
            p95_absolute_depth_error_m=float(np.percentile(errors,95)) if len(errors) else None))
    sequence.close()
    report=dict(kind='offline_actual_stereo_matching_not_flight_qualification',records=records,
        frames=len(records),median_wall_s=float(np.median([r['wall_s'] for r in records])),
        ideal_depth_used_by_matcher=False,reference_scope='sparse reprojected ideal-center depth, rounded pixels; diagnostic occlusion/sampling limits',
        guidance_scope=guidance['scope'] if guidance else None,
        guidance_sha256=sha256(args.guidance.read_bytes()).hexdigest() if args.guidance else None,
        airframe_contract_sha256=sha256(args.airframe_contract.read_bytes()).hexdigest() if args.airframe_contract else None,
        input_manifest_sha256=sha256((args.evidence/'pairs.json').read_bytes()).hexdigest())
    (args.output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2))


if __name__=='__main__':main()
