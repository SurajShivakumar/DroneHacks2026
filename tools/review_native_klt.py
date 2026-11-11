"""Offline stereo-initializer diagnosis; no ROS, pose prior or flight output.

Reproduces the pinned KLT parameters on fresh FAST/grid detections, not the
historical frontend's occupied masks or exact live feature IDs. Multi-start
disagreement is evidence of initializer sensitivity, not proof of which depth
is correct. Images and timestamps are recorded sensor data only.
"""
import argparse,json
from pathlib import Path
from hashlib import sha256
import numpy as np


def preprocess(image,method):
    import cv2
    if method=='CLAHE':return cv2.createCLAHE(clipLimit=10.,tileGridSize=(8,8)).apply(image)
    if method=='HISTOGRAM':return cv2.equalizeHist(image)
    if method=='NONE':return image.copy()
    raise ValueError('explicit histogram method required')


def fresh_fast_grid(image):
    import cv2
    h,w=image.shape;grid_x,grid_y=8,5;points=[]
    detector=cv2.FastFeatureDetector_create(threshold=10,nonmaxSuppression=True)
    for x in range(grid_x):
        for y in range(grid_y):
            sx,sy=w//grid_x,h//grid_y
            selected=sorted(detector.detect(image[y*sy:(y+1)*sy,x*sx:(x+1)*sx]),key=lambda p:p.response,reverse=True)[:11]
            points.extend([(p.pt[0]+x*sx,p.pt[1]+y*sy) for p in selected])
    if not points:return np.empty((0,2),np.float32)
    uv=np.asarray(points,np.float32)
    cv2.cornerSubPix(image,uv,(5,5),(-1,-1),(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,20,.001))
    occupied=set();out=[]
    for p in uv:
        cell=tuple((p/8).astype(int))
        if cell not in occupied and 10<=p[0]<w-10 and 10<=p[1]<h-10:out.append(p);occupied.add(cell)
    return np.asarray(out,np.float32).reshape(-1,2)


def hypothesis(left,right,uv,seed_disparity,*,max_level=5):
    import cv2
    if len(uv)==0:return dict(valid=np.zeros(0,bool),disparity=np.empty(0),error=np.empty(0),epipolar=np.empty(0),fb=np.empty(0),native=np.zeros(0,bool))
    params=dict(winSize=(15,15),maxLevel=max_level,criteria=(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,30,.01),flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
    initial=uv.copy();initial[:,0]-=seed_disparity
    q,status,error=cv2.calcOpticalFlowPyrLK(left,right,uv,initial,**params)
    back,reverse,_=cv2.calcOpticalFlowPyrLK(right,left,q,uv.copy(),**params)
    h,w=left.shape
    native=status.ravel().astype(bool)&np.isfinite(q).all(axis=1)&(q[:,0]>=0)&(q[:,0]<w)&(q[:,1]>=0)&(q[:,1]<h)
    disparity=uv[:,0]-q[:,0];epi=abs(uv[:,1]-q[:,1]);fb=np.linalg.norm(uv-back,axis=1)
    valid=native&reverse.ravel().astype(bool)&(epi<=1)&(fb<=1)&(disparity>=.5)&(disparity<=128)
    return dict(valid=valid,disparity=disparity,error=error.ravel(),epipolar=epi,fb=fb,native=native)


def ambiguity_count(results):
    if not results:return 0
    count=0
    for index in range(len(results[0]['valid'])):
        rows=[(float(r['disparity'][index]),float(r['error'][index])) for r in results if r['valid'][index]]
        if not rows:continue
        best=min(e for _,e in rows)
        good=sorted(d for d,e in rows if e<=best*1.15+2)
        if good and good[-1]-good[0]>3:count+=1
    return count


def metrics(r):
    valid=r['valid'];native=r['native']
    quant=lambda x:dict(median=float(np.median(x)),p95=float(np.percentile(x,95))) if len(x) else None
    return dict(native_mask_oob_accepted=int(sum(native)),diagnostic_lr_epi_positive_accepted=int(sum(valid)),
        native_epipolar_error_px=quant(r['epipolar'][native]),native_fb_error_px=quant(r['fb'][native]),
        diagnostic_disparity_px=quant(r['disparity'][valid]),native_nonpositive_disparities=int(sum(native&(r['disparity']<.5))))


def main():
    import cv2
    cv2.setNumThreads(1)
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();data=json.loads((args.evidence/'pairs.json').read_text());reports=[]
    seeds=[0.,8.,16.,24.,32.,48.,64.,96.]
    for pair in data['pairs']:
        if abs(pair['left_stamp_ns']-pair['right_stamp_ns'])>10_000_000:raise ValueError('unsynchronized pair')
        images=[]
        for side in ('left','right'):
            path=args.evidence/pair[side]
            if sha256(path.read_bytes()).hexdigest()!=pair[side+'_sha256']:raise ValueError('image hash mismatch')
            images.append(cv2.imread(str(path),cv2.IMREAD_GRAYSCALE))
        for method in ('CLAHE','HISTOGRAM','NONE'):
            left,right=[preprocess(image,method) for image in images];uv=fresh_fast_grid(left)
            results=[hypothesis(left,right,uv,seed) for seed in seeds]
            reports.append(dict(target_s=pair['target_s'],source_s=pair['left_stamp_ns']/1e9,method=method,points=len(uv),
                seeds={str(seed):metrics(r) for seed,r in zip(seeds,results)},comparable_cost_disparity_ambiguous_points=ambiguity_count(results),
                zero_seed_samples=[dict(uv=point.tolist(),native=bool(results[0]['native'][i]),valid=bool(results[0]['valid'][i]),
                    **{k:float(results[0][k][i]) if np.isfinite(results[0][k][i]) else None for k in ('disparity','error','epipolar','fb')}) for i,point in enumerate(uv)]))
    result=dict(kind='offline_native_klt_initializer_sensitivity_not_depth_truth_or_flight',opencv_version=cv2.__version__,numpy_version=np.__version__,
        script_sha256=sha256(Path(__file__).read_bytes()).hexdigest(),pairs_sha256=sha256((args.evidence/'pairs.json').read_bytes()).hexdigest(),
        native_parameters=dict(window=[15,15],pyramid_levels=5,iterations=30,epsilon=.01,initial_flow=True,clahe_clip=10,clahe_tiles=[8,8]),
        diagnostic_added_checks=dict(epipolar_px=1,forward_backward_px=1,positive_disparity_range_px=[.5,128],ambiguity_separation_px=3,cost_relative=.15,cost_absolute=2),
        reports=reports,limitations='Fresh-frame grid detections omit live occupied masks/history. No scene truth, ideal depth or map priors used. LR consistency and matching cost do not certify depth. Alternate histogram results use different detections. No native frontend or flight code changed.')
    with args.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k!='reports'},indent=2))
    for r in reports:print(json.dumps({k:v for k,v in r.items() if k not in ('seeds','zero_seed_samples')}|{'zero_seed':r['seeds']['0.0']}))


if __name__=='__main__':main()
