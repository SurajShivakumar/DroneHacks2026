"""Offline read-only shadow audit. Scene poses enter only this evaluation tool.

Processed frames are not position fixes: a frame may have zero usable wall
factors. This cannot establish course completion or hardware qualification.
"""
import argparse,json,sys
from pathlib import Path
from hashlib import sha256
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def summarize(rows):
    if not rows:raise ValueError('no processed shadow frames')
    stamps=np.array([r['source_s'] for r in rows])
    if not np.isfinite(stamps).all() or np.any(np.diff(stamps)<=0):raise ValueError('source resets/nonadvancing frames')
    for r in rows:
        if not np.isfinite(r['right_source_s']) or abs(r['right_source_s']-r['source_s'])>.01:raise ValueError('stereo skew exceeded')
        for key,t in r['support_source_s'].items():
            if not 0<=r['source_s']-t<=(.05 if key=='vio' else .1):raise ValueError('noncausal or stale support')
        if not np.isfinite(r['processing_wall_s']) or not 0<r['processing_wall_s']<=.1:raise ValueError('invalid/excessive processing time')
    times=np.array([r['processing_wall_s'] for r in rows]); factors=np.array([r['accepted_wall_factors'] for r in rows])
    report=dict(kind='offline_live_shadow_log_audit_not_flight_qualification',processed_frames=len(rows),
        source_interval_s=[float(stamps[0]),float(stamps[-1])],max_processed_source_gap_s=float(np.diff(stamps).max()) if len(rows)>1 else None,
        processing_wall_ms=dict(median=float(np.median(times)*1000),p95=float(np.percentile(times,95)*1000),maximum=float(times.max()*1000)),
        median_valid_stereo_points=float(np.median([r['sensor']['valid_points'] for r in rows])),
        frames_with_accepted_wall_factor=int(sum(factors>0)),frames_without_wall_factor=int(sum(factors==0)),
        total_shadow_fused_factors=sum(r.get('wall_factors_fused',0) for r in rows),published_flight_odometry=False)
    if all(r.get('covariance_xy_m2') is not None for r in rows):
        std=[]; rank_two=0
        for row in rows:
            C=np.asarray(row['covariance_xy_m2'],float)
            if (C.shape!=(2,2) or not np.isfinite(C).all() or not np.allclose(C,C.T,atol=1e-10,rtol=1e-7)
                    or np.linalg.eigvalsh(C).min()<-1e-12):raise ValueError('invalid shadow XY covariance')
            std.append(float(np.sqrt(max(np.linalg.eigvalsh(C).max(),0))))
            normals=np.array([f['normal'] for f in row.get('factors',[]) if f['accepted']])
            if normals.size:
                if normals.ndim!=2 or normals.shape[1]!=2 or not np.isfinite(normals).all():raise ValueError('invalid wall normals')
                rank_two+=int(np.linalg.matrix_rank(normals,tol=.01)==2)
        report['reported_uncertainty_not_calibrated']=dict(median_largest_xy_std_m=float(np.median(std)),
            maximum_largest_xy_std_m=max(std),final_largest_xy_std_m=std[-1],frames_with_two_accepted_normal_directions=rank_two,
            scope='engineering covariance; accepted factor directions may still be rejected by the filter')
    return report


def evaluate(rows,poses):
    states=np.genfromtxt(poses,delimiter=',',names=True)
    t=np.atleast_1d(states['t']); points=np.stack([np.atleast_1d(states[k]) for k in ('x','y','z')],axis=1)
    if len(t)<2 or not np.isfinite(t).all() or not np.isfinite(points).all() or np.any(np.diff(t)<=0):
        raise ValueError('finite monotonic independent scene poses required')
    raw=[]; filtered=[]; height=[]; raw_vio=[]
    for row in rows:
        source=row['source_s']
        if not t[0]<=source<=t[-1]:raise ValueError('scene evidence does not cover shadow source')
        i=min(max(int(np.searchsorted(t,source,side='right'))-1,0),len(t)-2)
        if t[i+1]-t[i]>.1:raise ValueError('scene evidence gap exceeds .1s')
        u=(source-t[i])/(t[i+1]-t[i]); truth=points[i]*(1-u)+points[i+1]*u
        prior=np.array(row['position_prior_map_m'])
        raw.append(float(np.linalg.norm(prior[:2]-truth[:2])))
        if 'raw_vio_xy_m' in row:raw_vio.append(float(np.linalg.norm(np.array(row['raw_vio_xy_m'])-truth[:2])))
        height.append(float(prior[2]-truth[2]))
        if row.get('filtered_xy_m') is not None:filtered.append(float(np.linalg.norm(np.array(row['filtered_xy_m'])-truth[:2])))
    stats=lambda x:dict(rms_m=float(np.sqrt(np.mean(np.square(x)))),maximum_absolute_m=float(np.max(np.abs(x))))
    return dict(independent_scene_sha256=sha256(poses.read_bytes()).hexdigest(),
        prior_xy_error=stats(raw),filtered_xy_error=stats(filtered) if filtered else None,
        raw_vio_xy_error=stats(raw_vio) if raw_vio else None,
        range_height_error=stats(height),alignment='declared diagnostic initialization; no truth-fit alignment',
        scope='same source window; partial disarmed fixture only; no flight success or hardware claim')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observations',type=Path,required=True); parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--poses',type=Path); parser.add_argument('--source-window',nargs=2,type=float)
    args=parser.parse_args()
    rows=[json.loads(line) for line in args.observations.read_text().splitlines()]
    if args.source_window:
        lo,hi=args.source_window
        if not np.isfinite([lo,hi]).all() or lo>=hi:raise ValueError('finite increasing source window required')
        rows=[row for row in rows if lo<=row['source_s']<=hi]
    result=summarize(rows);result['observations_sha256']=sha256(args.observations.read_bytes()).hexdigest()
    result['source_window_requested_s']=args.source_window
    if args.poses:result['independent_evaluation']=evaluate(rows,args.poses)
    with args.output.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
