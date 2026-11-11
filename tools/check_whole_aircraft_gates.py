"""Independent whole-envelope check of teammate gate planes, not official grade."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dronebench.gate_geometry import whole_crossing


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('poses','repo','contract','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    sys.path.insert(0,str(args.repo.resolve()))
    from drl.config import load_config
    _,_,gates=load_config(str(args.repo.resolve()/'config/course_v3.yaml'))
    states=np.genfromtxt(args.poses,delimiter=',',names=True)
    times=states['t']; positions=np.stack([states[k] for k in ('x','y','z')],axis=-1)
    quaternions=np.stack([states[k] for k in ('qw','qx','qy','qz')],axis=-1)
    contract=json.loads(args.contract.read_text()); start=0; results=[]
    for i,name in enumerate(gates.names):
        if gates.is_hover[i]: continue
        result=whole_crossing(times,positions,quaternions,centre=gates.centre[i],
            normal=gates.normal[i],lateral=gates.lateral[i],half_width=gates.half_w[i],
            half_height=gates.half_h[i],radius=contract['collision_radius_m'],
            body_half_height=contract['collision_half_height_m'],start_index=start)
        results.append({'gate':name,**result})
        if not result['whole_crossed']: break
        start=result['sample_index']
    report={'kind':'independent_conservative_teammate_plane_check_not_official_checkpoints',
        'input_hashes':{str(p):sha256(p.read_bytes()).hexdigest() for p in (args.poses,args.contract)},
        'gates':results,'all_nine_whole_crossed_in_order':len(results)==9 and all(x['whole_crossed'] for x in results),
        'all_conservative_rectangle_checks_passed':len(results)==9 and all(x.get('conservative_rectangle_clearance') for x in results),
        'caveat':'The teammate corner planes are navigation gates, not supplied official checkpoint-line geometry. Elliptical tunnel uses separate mesh checks. No landing/contact/course grade is inferred.'}
    with args.output.open('x',encoding='utf-8') as stream: json.dump(report,stream,indent=2)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
