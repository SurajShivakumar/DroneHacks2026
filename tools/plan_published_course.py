"""A* + checked shortcuts from the PUBLISHED layout, not simulator truth.

Reuse teammate OBJ primitives and project A*. Output a separate classical
stop/turn/go reference, with a fixed altitude and explicit speed/acceleration/
jerk limits. Independent mesh clearance must pass before any flight.
"""
import argparse
from hashlib import sha256
import json
from math import atan2,ceil,cos,pi,sin,sqrt
from pathlib import Path
import sys
import time
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dronebench.grid import OccupancySlice,find_path
from dronebench.path_smoothing import shortcut_path,path_length


def sampled_reference(points, *, altitude=.9,cruise=.25,acceleration=.15,jerk=.3,yaw_rate=.15,dt=.02):
    rows=[];now=0.;yaw=pi/2
    def row(position,velocity,angle):return [now,*position,*velocity,cos(angle/2),0,0,sin(angle/2)]
    position=np.r_[points[0],altitude];rows.append(row(position,np.zeros(3),yaw))
    for a,b in zip(points,points[1:]):
        a=np.r_[a,altitude];b=np.r_[b,altitude];delta=b-a;length=float(np.linalg.norm(delta))
        if length<1e-8:continue
        next_yaw=atan2(delta[1],delta[0]);difference=atan2(sin(next_yaw-yaw),cos(next_yaw-yaw))
        if abs(difference)>.01:
            turn=abs(difference)/yaw_rate
            count=ceil(turn/dt)
            for i in range(1,count+1):
                now+=turn/count;rows.append(row(a,np.zeros(3),yaw+difference*i/count))
            yaw+=difference
            for _ in range(ceil(.75/dt)):
                now+=dt;rows.append(row(a,np.zeros(3),yaw))
        duration=max(1.875*length/cruise,sqrt(5.774*length/acceleration),(60*length/jerk)**(1/3))
        count=ceil(duration/dt)
        for i in range(1,count+1):
            u=i/count;s=10*u**3-15*u**4+6*u**5;ds=(30*u*u-60*u**3+30*u**4)/duration
            now+=duration/count;rows.append(row(a+s*delta,ds*delta,yaw))
        for _ in range(ceil(.75/dt)):
            now+=dt;rows.append(row(b,np.zeros(3),yaw))
    return np.asarray(rows)


def build(repo,contract_path,output):
    output.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(repo.resolve()))
    from drl.config import load_config
    config,course,gates=load_config(str(repo.resolve()/'config/course_v3.yaml'))
    contract=json.loads(contract_path.read_text());radius=contract['collision_radius_m']+.08
    altitude=.9;resolution=.03;width=height=204;origin=-3.06
    xx,yy=np.meshgrid(origin+(np.arange(width)+.5)*resolution,origin+(np.arange(height)+.5)*resolution)
    centres=np.c_[xx.ravel(),yy.ravel(),np.full(width*height,altitude)]
    occupied=course.collides(centres,0,contract['collision_half_height_m']+.08)
    grid=OccupancySlice(width,height,resolution,tuple(100 if value else 0 for value in occupied),origin,origin)
    window=gates.names.index('window');normal=gates.normal[window,:2]
    targets=[np.array([-2.515,.914]),*[p[:2] for p in gates.centre[:6]],
        gates.centre[6,:2]+[0,.5],gates.centre[6,:2],gates.centre[7,:2],
        gates.centre[window,:2]-.6*normal,gates.centre[window,:2]+.6*normal,gates.centre[-1,:2]]
    route=[tuple(targets[0])];legs=[];started=time.perf_counter()
    for i,(a,b) in enumerate(zip(targets,targets[1:])):
        start=grid.cell_to_world(*grid.world_to_cell(*a));goal=grid.cell_to_world(*grid.world_to_cell(*b))
        path=find_path(grid,start,goal,radius)
        if path is None:raise ValueError(f'published-map A* leg {i} has no conservatively clear path')
        shortened=shortcut_path(grid,path,radius,start=a,goal=b)
        route.extend(shortened[1:]);legs.append({'index':i,'raw_points':len(path),'shortened_points':len(shortened),
                                               'length_m':path_length(shortened)})
        print(f'Checked published-map leg {i}: {len(path)} -> {len(shortened)} points',flush=True)
    rows=sampled_reference(route,altitude=altitude)
    np.savetxt(output/'trajectory.csv',rows,delimiter=',',header='t,x,y,z,vx,vy,vz,qw,qx,qy,qz',comments='')
    obj=repo/config['course_obj']
    report={'kind':'published_layout_classical_astar_plan_not_flight','source_obj_sha256':sha256(obj.read_bytes()).hexdigest(),
        'teammate_config_sha256':sha256((repo/'config/course_v3.yaml').read_bytes()).hexdigest(),
        'airframe_contract_sha256':sha256(contract_path.read_bytes()).hexdigest(),
        'generator_sha256':sha256(Path(__file__).read_bytes()).hexdigest(),
        'map_source':'published OBJ prior; no simulator state, recorded flight map or sensor-derived map',
        'map_resolution_m':resolution,'planning_xy_radius_m':radius,'planning_z_half_height_m':contract['collision_half_height_m']+.08,
        'altitude_m':altitude,'cruise_mps':.25,'acceleration_mps2':.15,'jerk_mps3':.3,'yaw_rate_rad_s':.15,
        'legs':legs,'waypoints_xy_m':route,'length_m':path_length(route),'reference_duration_s':float(rows[-1,0]),
        'planning_wall_s':time.perf_counter()-started,'requires_independent_mesh_screen':True,
        'flight_completed':False,'finish_time_s':None}
    (output/'plan.json').write_text(json.dumps(report,indent=2));return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('repo','contract','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();report=build(args.repo,args.contract,args.output)
    print(json.dumps({k:v for k,v in report.items() if k not in ('legs','waypoints_xy_m')},indent=2))
