"""Independent OFFLINE evaluation of already-saved image-only KLT outputs.

Scene poses are read only here, after matching has finished. Net-junction
classification is an image heuristic, not semantic ground truth; report it
separately and never treat this tool's expected disparity as a flight prior.
"""
import argparse,json,sys
from pathlib import Path
from hashlib import sha256
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.compare_vio_scene import scene_pose


def grid_junction_mask(image,uv):
    import cv2
    dark=(image<5).astype(np.uint8)*255
    lines=cv2.HoughLinesP(dark,1,np.pi/720,threshold=70,minLineLength=min(image.shape)*.55,maxLineGap=8)
    families=[[],[]]
    for line in lines.reshape(-1,4) if lines is not None else []:
        a=np.array(line[:2],float);b=np.array(line[2:],float);delta=b-a;length=np.linalg.norm(delta)
        if length==0:continue
        if abs(delta[0])/length<.035:family=0
        elif abs(delta[1])/length<.035:family=1
        else:continue
        normal=np.array([-delta[1],delta[0]])/length
        families[family].append(abs((uv-a)@normal))
    distances=[np.min(group,axis=0) if group else np.full(len(uv),np.inf) for group in families]
    return (distances[0]<=2.5)&(distances[1]<=2.5),dict(vertical_lines=len(families[0]),horizontal_lines=len(families[1]))


def main():
    import cv2
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('evidence','poses','contract','course-manifest','arena-contract','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    # This file contains the saved matcher outputs generated without truth.
    saved=json.loads((args.evidence/'result-v2.json').read_text())
    pairs={r['target_s']:r for r in json.loads((args.evidence/'pairs.json').read_text())['pairs']}
    truth=np.genfromtxt(args.poses,delimiter=',',names=True)
    states=np.stack([truth[k] for k in ('t','x','y','z','qw','qx','qy','qz')],axis=1)
    chain=yaml.safe_load((args.evidence/'calibration.yaml').read_text().replace('%YAML:1.0',''))
    body_from_imu=np.asarray(json.loads(args.contract.read_text())['body_from_imu'])
    T=body_from_imu@np.asarray(chain['cam0']['T_imu_cam'])
    right=body_from_imu@np.asarray(chain['cam1']['T_imu_cam'])
    separation=T[:3,:3].T@(right[:3,3]-T[:3,3]);baseline=float(separation[0])
    if baseline<=0 or not np.allclose(separation[1:],0,atol=1e-9):raise ValueError('positive rectified stereo baseline required')
    fx,fy,cx,cy=chain['cam0']['intrinsics']
    course=json.loads(args.course_manifest.read_text())['geometry']['groups'];north=course['Net side north']
    plane=(north['min'][1]+north['max'][1])/2
    arena=json.loads(args.arena_contract.read_text())
    bg_north=next(r for r in arena['added_visual_surfaces'] if r['name']=='north')['vertices_m'][0][0][1]
    reports=[]
    for result in saved['reports']:
        if result['method']!='CLAHE':continue
        pair=pairs[result['target_s']];image=cv2.imread(str(args.evidence/pair['left']),0)
        samples=result['zero_seed_samples'];uv=np.array([r['uv'] for r in samples])
        valid=np.array([r['valid'] for r in samples]);d=np.array([r['disparity'] for r in samples])
        junction,counts=grid_junction_mask(image,uv)
        t=result['source_s']
        if not truth['t'][0]<=t<=truth['t'][-1]:raise ValueError('independent scene coverage missing')
        body_p,body_R=scene_pose(truth['t'],states,t)
        origin=body_p+body_R@T[:3,3];camera_R=body_R@T[:3,:3]
        rays=np.c_[(uv[:,0]-cx)/fx,(uv[:,1]-cy)/fy,np.ones(len(uv))]@camera_R.T
        net_z=(plane-origin[1])/rays[:,1];bg_z=(bg_north-origin[1])/rays[:,1]
        hits=origin+rays*net_z[:,None]
        front=(net_z>0)&(hits[:,0]>=north['min'][0])&(hits[:,0]<=north['max'][0])&(hits[:,2]>=0)&(hits[:,2]<=north['max'][2])
        picked=valid&junction&front
        net_d=fx*baseline/net_z;bg_d=fx*baseline/bg_z
        pitch_x=(north['max'][0]-north['min'][0])/np.ceil((north['max'][0]-north['min'][0])/.05)
        period=net_d*pitch_x/baseline
        stats=lambda x:dict(median=float(np.median(x)),minimum=float(np.min(x)),maximum=float(np.max(x))) if len(x) else None
        reports.append(dict(source_s=t,body_position_evaluation_m=body_p.tolist(),camera_origin_evaluation_m=origin.tolist(),
            north_net_world_y_m=plane,background_north_world_y_m=bg_north,
            all_diagnostic_valid=int(sum(valid)),black_grid_junction_candidates=int(sum(junction)),evaluated_valid_front_net_junctions=int(sum(picked)),
            optical_net_z_m=stats(net_z[picked]),optical_background_z_m=stats(bg_z[picked]),
            observed_disparity_px=stats(d[picked]),expected_net_disparity_px=stats(net_d[picked]),expected_background_disparity_px=stats(bg_d[picked]),
            error_in_net_grid_periods=stats((net_d[picked]-d[picked])/period[picked]),
            junctions_more_than_2px_from_net=int(sum(abs(net_d[picked]-d[picked])>2)),hough=counts,
            samples=[dict(uv=uv[i].tolist(),observed_disparity_px=float(d[i]),expected_net_disparity_px=float(net_d[i]),expected_background_disparity_px=float(bg_d[i])) for i in np.flatnonzero(picked)]))
    report=dict(kind='offline_independent_net_junction_evaluation_not_matcher_prior',reports=reports,
        evaluator_sha256=sha256(Path(__file__).read_bytes()).hexdigest(),
        saved_matcher_sha256=sha256((args.evidence/'result-v2.json').read_bytes()).hexdigest(),
        poses_sha256=sha256(args.poses.read_bytes()).hexdigest(),
        calibration_sha256=sha256((args.evidence/'calibration.yaml').read_bytes()).hexdigest(),
        contract_sha256=sha256(args.contract.read_bytes()).hexdigest(),
        course_manifest_sha256=sha256(args.course_manifest.read_bytes()).hexdigest(),
        arena_contract_sha256=sha256(args.arena_contract.read_bytes()).hexdigest(),
        limitations='Black-line intersection is a heuristic; may misclassify background corners projected near net crossings. Net expected disparity uses strand midplane, ignoring1.5mm half-thickness. No ideal depth used. Does not recover actual live feature IDs, occupied masks, track ages, rejection reasons or prove a complete causal explanation.')
    with args.output.open('x') as f:json.dump(report,f,indent=2,allow_nan=False)
    for r in reports:print(json.dumps({k:v for k,v in r.items() if k!='samples'},indent=2))


if __name__=='__main__':main()
