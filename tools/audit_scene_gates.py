"""Audit recorded scene poses against published route planes, offline only.

This uses teammate gate geometry but an independent full-envelope crossing
algorithm. It does not establish official checkpoint scoring or a course finish.
"""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dronebench.gate_audit import audit


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('repo','poses','contract','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    sys.path.insert(0,str(args.repo.resolve()))
    from drl.config import load_config
    _,_,resolved=load_config(str(args.repo.resolve()/'config/course_v3.yaml'))
    gates=[dict(name=resolved.names[i],center=resolved.centre[i].tolist(),
                normal=resolved.normal[i].tolist(),half_width=float(resolved.half_w[i]),
                half_height=float(resolved.half_h[i]))
           for i in range(len(resolved)) if not resolved.is_hover[i]]
    contract=json.loads(args.contract.read_text())
    samples=np.genfromtxt(args.poses,delimiter=',',names=True)
    rows=np.stack([samples[k] for k in ('t','x','y','z','qw','qx','qy','qz')],axis=1)
    result=audit(rows,gates,radius=contract['collision_radius_m'],
                 half_height=contract['collision_half_height_m'])
    result.update(kind='offline_sampled_route_plane_audit_not_official_score',
                  course_finish_verified=False,landing_disarm_verified=False,
                  mesh_contacts_evaluated=False,gates=gates,
                  input_sha256={str(p):sha256(p.read_bytes()).hexdigest()
                    for p in (args.poses,args.contract,args.repo/'config/course_v3.yaml',
                              Path(__file__),Path(__file__).resolve().parents[1]/'dronebench/gate_audit.py')})
    with args.output.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
