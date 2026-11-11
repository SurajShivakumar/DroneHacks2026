"""Offline local gate-normal corner curves; no ROS or simulator state.

Each splice, gate centre and ordinary waypoint is at rest. This avoids moving
through curvature discontinuities while crossing diagonal markers normally.
Geometry and enlarged-envelope gate checks remain independent and unchanged.
"""
import argparse,json,sys,shutil
from pathlib import Path
from hashlib import sha256
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


def bezier(P,u):
    P=np.asarray(P,float);u=np.asarray(u,float)[:,None];v=1-u
    return (v**3*P[0]+3*v*v*u*P[1]+3*v*u*u*P[2]+u**3*P[3],
        3*v*v*(P[1]-P[0])+6*v*u*(P[2]-P[1])+3*u*u*(P[3]-P[2]),
        6*v*(P[2]-2*P[1]+P[0])+6*u*(P[3]-2*P[2]+P[1]),
        np.broadcast_to(6*(P[3]-3*P[2]+3*P[1]-P[0]),(len(u),2)))


def unit_motion(P,u):
    u=np.asarray(u,float);s=10*u**3-15*u**4+6*u**5
    d=30*u*u-60*u**3+30*u**4;dd=60*u-180*u*u+120*u**3;ddd=60-360*u+360*u*u
    p,a,b,c=bezier(P,s)
    return p,a*d[:,None],b*d[:,None]**2+a*dd[:,None],c*d[:,None]**3+3*b*(d*dd)[:,None]+a*ddd[:,None]


def corner_segments(points,gates,*,splice=.55,handle=.28):
    points=np.asarray(points,float)
    if (points.ndim!=2 or points.shape[1]!=2 or len(points)<3 or not np.isfinite(points).all()
            or not np.isfinite([splice,handle]).all() or not .3<=splice<=.6 or not .08<=handle<=.3):
        raise ValueError('finite public polyline and bounded local splice/handle required')
    corners={}
    for i in range(1,len(points)-1):
        matched=[g for g in gates if np.linalg.norm(points[i]-np.asarray(g['center'])[:2])<1e-6 and abs(g['normal'][0])>.1 and abs(g['normal'][1])>.1]
        if not matched:continue
        if len(matched)!=1:raise ValueError('ambiguous public gate waypoint')
        incoming=points[i]-points[i-1];outgoing=points[i+1]-points[i]
        if min(np.linalg.norm(incoming),np.linalg.norm(outgoing))<=splice:raise ValueError('insufficient straight splice length')
        incoming/=np.linalg.norm(incoming);outgoing/=np.linalg.norm(outgoing)
        normal=np.asarray(matched[0]['normal'],float)[:2]
        if abs(np.linalg.norm(normal)-1)>1e-8 or min(incoming@normal,outgoing@normal)<.5:raise ValueError('gate normal inconsistent with ordered traversal')
        C=points[i];A=C-splice*incoming;B=C+splice*outgoing
        corners[i]=dict(A=A,B=B,C=C,normal=normal,name=matched[0]['name'])
    for i in corners:
        if i+1 in corners and np.linalg.norm(points[i+1]-points[i])<2*splice:raise ValueError('adjacent corner splices overlap')
    pieces=[];current=points[0]
    def line(a,b):return np.array([a,a+(b-a)/3,a+2*(b-a)/3,b])
    for i in range(1,len(points)):
        if i in corners:
            c=corners[i];A,B,C,n=(c[k] for k in ('A','B','C','normal'))
            if np.linalg.norm(A-current)>1e-8:pieces.append((line(current,A),'line_to_splice',None))
            # Outer controls coincide with splice endpoints: every splice is
            # stopped, so no moving-curvature or tangent jump is introduced.
            pieces.append((np.array([A,A,C-handle*n,C]),'gate_approach',c['name']))
            pieces.append((np.array([C,C+handle*n,B,B]),'gate_departure',c['name']))
            current=B
        else:
            if np.linalg.norm(points[i]-current)>1e-8:pieces.append((line(current,points[i]),'line_to_waypoint',None))
            current=points[i]
    return pieces,corners


def sample(pieces,*,altitude=.9,speed=.267,acceleration=.111,jerk=.148,dt=.015):
    if not np.isfinite([altitude,speed,acceleration,jerk,dt]).all() or min(altitude,speed,acceleration,jerk,dt)<=0 or dt>.02:
        raise ValueError('positive finite reviewed motion bounds and dense sampling required')
    rows=[];now=0.;reports=[];q=[np.sqrt(.5),0.,0.,np.sqrt(.5)]
    for P,kind,name in pieces:
        probe=unit_motion(P,np.linspace(0,1,4001));limits=[np.linalg.norm(a,axis=1).max() for a in probe[1:]]
        duration=1.01*max(limits[0]/speed,np.sqrt(limits[1]/acceleration),(limits[2]/jerk)**(1/3))
        count=int(np.ceil(duration/dt));u=np.linspace(0,1,count+1);p,v,a,j=unit_motion(P,u)
        if not rows:rows.append([now,*p[0],altitude,0.,0.,0.,*q])
        for k in range(1,count+1):rows.append([now+duration*u[k],*p[k],altitude,*(v[k]/duration),0.,*q])
        reports.append(dict(kind=kind,gate=name,duration_s=float(duration),controls_xy_m=np.asarray(P).tolist(),
            peak_speed_mps=float(np.linalg.norm(v,axis=1).max()/duration),
            peak_acceleration_mps2=float(np.linalg.norm(a,axis=1).max()/duration**2),
            peak_jerk_mps3=float(np.linalg.norm(j,axis=1).max()/duration**3)))
        now+=duration
        # A short explicit stop also makes splice/gate rest reviewable.
        hold=.25;num=int(np.ceil(hold/dt))
        for k in range(1,num+1):rows.append([now+hold*k/num,*p[-1],altitude,0.,0.,0.,*q])
        now+=hold
    return np.asarray(rows),reports


def build(source_plan,gate_audit,contract,output,*,splice=.55,handle=.28):
    source_plan,gate_audit,contract,output=map(Path,(source_plan,gate_audit,contract,output))
    plan=json.loads(source_plan.read_text());gates=json.loads(gate_audit.read_text())['gates'];airframe=json.loads(contract.read_text())
    if plan['kind']!='published_layout_classical_astar_plan_not_flight' or plan['airframe_contract_sha256']!=sha256(contract.read_bytes()).hexdigest():raise ValueError('matching public-map source/aircraft contract required')
    pieces,corners=corner_segments(plan['waypoints_xy_m'],gates,splice=splice,handle=handle)
    rows,reports=sample(pieces,altitude=plan['altitude_m'])
    output.mkdir(parents=True,exist_ok=False)
    np.savetxt(output/'trajectory.csv',rows,delimiter=',',header='t,x,y,z,vx,vy,vz,qw,qx,qy,qz',comments='')
    inflated=dict(airframe);inflated['collision_radius_m']+=.08;inflated['collision_half_height_m']+=.08
    (output/'gate-reserve-08-contract.json').write_text(json.dumps(inflated,indent=2))
    report=dict(kind='offline_local_gate_normal_corner_reference_not_flight',
        source_hashes={str(p):sha256(p.read_bytes()).hexdigest() for p in (source_plan,gate_audit,contract)},
        generator_sha256=sha256(Path(__file__).read_bytes()).hexdigest(),splice_m=splice,handle_m=handle,
        modified_gates=[c['name'] for c in corners.values()],segments=reports,
        reference_duration_s=float(rows[-1,0]),required_caller_time_scale=1.,reference_peak_speed_mps=max(r['peak_speed_mps'] for r in reports),
        flight_completed=False,finish_time_s=None,clearance_and_reserve_gate_checks_pending=True,
        limitations='Local curve/retiming candidate only, no live estimator or flight. Every splice and gate centre stops; fixed north yaw and 0.9m altitude. 8cm gate reserve is expressed through a separate enlarged diagnostic contract; source aircraft/physical geometry and checker thresholds remain unchanged. No competition-time claim.')
    (output/'plan.json').write_text(json.dumps(report,indent=2));shutil.copy2(Path(__file__),output/'generator.py')
    for path,name in ((source_plan,'source-plan.json'),(gate_audit,'source-gates.json'),(contract,'airframe-contract.json')):shutil.copy2(path,output/name)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('source-plan','gate-audit','contract','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--splice',type=float,default=.55);p.add_argument('--handle',type=float,default=.28)
    a=p.parse_args();r=build(a.source_plan,a.gate_audit,a.contract,a.output,splice=a.splice,handle=a.handle)
    print(json.dumps({k:v for k,v in r.items() if k!='segments'},indent=2))
