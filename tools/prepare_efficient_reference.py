"""Offline retiming of the existing public-map A* route; no flight interface.

Preserves every checked waypoint and the published altitude. A multirotor can
translate sideways: path direction need not mandate body yaw. This candidate
holds the declared initial heading, brakes to rest at every corner, and uses
bounded quintic speed/acceleration/jerk. Fixed yaw is NOT a qualified perception
policy: live visibility and estimator checks must precede any use in flight.
"""
import argparse
from dataclasses import asdict,dataclass
from hashlib import sha256
import json
from math import ceil,cos,sin,sqrt
from pathlib import Path
import shutil
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))


@dataclass(frozen=True)
class MotionBudget:
    cruise_mps: float=.4
    acceleration_mps2: float=.25
    jerk_mps3: float=.5
    dwell_s: float=.25
    dt_s: float=.02
    fixed_yaw_rad: float=np.pi/2

    def __post_init__(self):
        if not np.isfinite(list(asdict(self).values())).all():raise ValueError('finite motion settings required')
        if min(self.cruise_mps,self.acceleration_mps2,self.jerk_mps3,self.dt_s)<=0 or self.dwell_s<0:
            raise ValueError('positive motion bounds and nonnegative dwell required')
        if self.dt_s>.025:raise ValueError('reference source sampling must support the strict gate audit')


def sample_braked_route(points,*,altitude=.9,budget=None):
    budget=budget or MotionBudget()
    if not isinstance(budget,MotionBudget):raise ValueError('validated motion budget required')
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=2 or len(points)<2 or not np.isfinite(points).all() or not np.isfinite(altitude) or altitude<=0:
        raise ValueError('finite XY route and positive altitude required')
    if np.any(np.linalg.norm(np.diff(points,axis=0),axis=1)<1e-6):raise ValueError('distinct successive route points required')
    now=0.;rows=[];legs=[];q=[cos(budget.fixed_yaw_rad/2),0.,0.,sin(budget.fixed_yaw_rad/2)]
    rows.append([now,*points[0],altitude,0.,0.,0.,*q])
    for index,(a,b) in enumerate(zip(points,points[1:])):
        delta=np.r_[b-a,0.]; length=float(np.linalg.norm(delta))
        # Exact peak derivatives of s(u)=10u^3-15u^4+6u^5 on [0,1].
        duration=max(1.875*length/budget.cruise_mps,
                     sqrt((10/np.sqrt(3))*length/budget.acceleration_mps2),
                     (60*length/budget.jerk_mps3)**(1/3))
        count=ceil(duration/budget.dt_s); beginning=now
        for i in range(1,count+1):
            u=i/count; s=10*u**3-15*u**4+6*u**5
            ds=(30*u*u-60*u**3+30*u**4)/duration
            now=beginning+duration*u
            rows.append([now,*(np.r_[a,altitude]+s*delta),*(ds*delta),*q])
        dwell_count=ceil(budget.dwell_s/budget.dt_s)
        for i in range(dwell_count):
            now+=budget.dwell_s/dwell_count
            rows.append([now,*b,altitude,0.,0.,0.,*q])
        legs.append(dict(index=index,length_m=length,movement_s=duration,dwell_s=budget.dwell_s,
                         peak_speed_mps=1.875*length/duration,
                         peak_acceleration_mps2=(10/np.sqrt(3))*length/duration**2,
                         peak_jerk_mps3=60*length/duration**3))
    return np.asarray(rows),legs


def build(source_plan,contract,output,*,budget=None):
    budget=budget or MotionBudget()
    plan=json.loads(source_plan.read_text())
    if plan.get('kind')!='published_layout_classical_astar_plan_not_flight':raise ValueError('public-map A* source plan required')
    if plan['airframe_contract_sha256']!=sha256(contract.read_bytes()).hexdigest():raise ValueError('source route was planned for a different aircraft contract')
    rows,legs=sample_braked_route(plan['waypoints_xy_m'],altitude=plan['altitude_m'],budget=budget)
    output.mkdir(parents=True,exist_ok=False)
    np.savetxt(output/'trajectory.csv',rows,delimiter=',',header='t,x,y,z,vx,vy,vz,qw,qx,qy,qz',comments='')
    # Reserves are engineering scheduling assumptions, not measured mission times.
    outbound_launch_land_reserve=30.;return_bonus_reserve=40.
    report=dict(kind='offline_public_route_retimming_not_flight_qualification',
        source_plan_sha256=sha256(source_plan.read_bytes()).hexdigest(),
        generator_sha256=sha256(Path(__file__).read_bytes()).hexdigest(),
        airframe_contract_sha256=sha256(contract.read_bytes()).hexdigest(),
        published_obj_sha256=plan['source_obj_sha256'],route_waypoints_xy_m=plan['waypoints_xy_m'],
        source_altitude_m=plan['altitude_m'],source_reference_duration_s=plan['reference_duration_s'],
        reference_duration_s=float(rows[-1,0]),motion=asdict(budget),legs=legs,
        scheduling_assumptions_s=dict(competition_limit=180.,outbound_launch_landing_reserve=outbound_launch_land_reserve,
            complete_return_bonus_reserve=return_bonus_reserve,
            remaining_margin=180.-rows[-1,0]-outbound_launch_land_reserve-return_bonus_reserve),
        path_geometry_changed=False,stationary_yaw_turns=0,flight_completed=False,finish_time_s=None,
        requires_independent_mesh_and_gate_audits=True,
        limitations='All legs stop at vertices; fixed north yaw is unqualified for camera visibility. Timing is reference-only with assumed reserves. Does not include actual launch/landing, tracking pauses, a checked return route, contacts, official scoring, or hardware dynamics.')
    (output/'plan.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    shutil.copy2(source_plan,output/'source-plan.json');shutil.copy2(contract,output/'airframe-contract.json')
    shutil.copy2(Path(__file__),output/'generator.py')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('source-plan','contract','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();result=build(args.source_plan,args.contract,args.output)
    print(json.dumps({k:v for k,v in result.items() if k not in ('legs','route_waypoints_xy_m')},indent=2))
