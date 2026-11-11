"""Offline Sky Beta envelope/reference clearance check; no flight commands."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def check(scene, trajectory, contract_path, output, tracking_margin=0., start_s=None,end_s=None):
    if not np.isfinite(tracking_margin) or tracking_margin<0:
        raise ValueError('nonnegative finite tracking margin required')
    contract=json.loads(contract_path.read_text())
    samples=np.genfromtxt(trajectory,delimiter=',',names=True)
    states=np.stack([samples[key] for key in ('t','x','y','z','qw','qx','qy','qz')],axis=-1)
    if states.ndim!=2 or len(states)<2 or not np.isfinite(states).all() or np.any(np.diff(states[:,0])<=0):
        raise ValueError('complete chronological finite pose samples required')
    start_s=states[0,0] if start_s is None else start_s
    end_s=states[-1,0] if end_s is None else end_s
    if not states[0,0]<=start_s<end_s<=states[-1,0]:
        raise ValueError('requested interval must be covered by actual pose samples')
    tree=ET.parse(scene); body=tree.find("worldbody/body[@name='sky_beta']")
    for geom in body.findall('geom'):
        geom.set('contype','0'); geom.set('conaffinity','0')
    ET.SubElement(body,'geom',name='estimated_sky_beta_envelope',type='cylinder',
        size=f"{contract['collision_radius_m']+tracking_margin} {contract['collision_half_height_m']+tracking_margin}",
        contype='1',conaffinity='1',mass='0')
    model=mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(),encoding='unicode'))
    data=mujoco.MjData(model)
    joint=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'sky_beta_free')
    address=model.jnt_qposadr[joint]
    envelope=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'estimated_sky_beta_envelope')
    count=0; worst=0.; contacts=[]
    for t in np.arange(start_s,end_s+1e-9,.005):
        pose=np.array([np.interp(t,states[:,0],states[:,i]) for i in range(1,8)])
        pose[3:]/=np.linalg.norm(pose[3:])
        data.qpos[address:address+7]=pose; mujoco.mj_forward(model,data)
        for contact in data.contact:
            if envelope in (contact.geom1,contact.geom2) and contact.dist < -1e-5:
                count+=1;worst=max(worst,float(-contact.dist))
                if len(contacts)<100:
                    other=contact.geom2 if contact.geom1==envelope else contact.geom1
                    contacts.append({'reference_s':float(t),'geom':mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,other),
                                     'penetration_m':float(-contact.dist)})
    result={'kind':'reference_clearance_screen_not_flight_or_course_grade',
        'input_hashes':{str(p):sha256(p.read_bytes()).hexdigest() for p in (scene,trajectory,contract_path)},
        'radius_m':contract['collision_radius_m'],'half_height_m':contract['collision_half_height_m'],
        'tracking_margin_m':tracking_margin,
        'source_interval_s':[float(start_s),float(end_s)],
        'sampling_dt_s':.005,'penetrating_contact_samples':count,'max_penetration_m':worst,
        'first_contacts':contacts,'reference_screen_passed':count==0,
        'limitations':'Tilted enclosing cylinder plus stated tracking margin versus convex-decomposed course mesh. Sampled reference only; no swept-volume proof, live flight, takeoff/landing or hardware qualification.'}
    with output.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in ('input_hashes','first_contacts')},indent=2))
    return count==0


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('scene','trajectory','contract','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--tracking-margin',type=float,default=0.)
    parser.add_argument('--start',type=float)
    parser.add_argument('--end',type=float)
    args=parser.parse_args()
    raise SystemExit(0 if check(args.scene,args.trajectory,args.contract,args.output,args.tracking_margin,args.start,args.end) else 1)
