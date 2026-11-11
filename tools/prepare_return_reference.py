"""Offline public-layout return-bonus geometry; no ROS or execution interface.

Rule7.10 permits crossing above the barrier only on the optional return after
landing/disarm. This begins and ends at airborne hover, excluding rearm,
takeoff, landing and actual bonus qualification. Outbound must first be verified.
"""
import argparse
from dataclasses import asdict
from hashlib import sha256
import json
from math import ceil,cos,sin,sqrt
from pathlib import Path
import shutil
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.prepare_efficient_reference import MotionBudget


def return_waypoints(settings,manifest,contract,*,tracking_margin=.08,extra_vertical_buffer=.03):
    if (not np.isfinite([tracking_margin,extra_vertical_buffer]).all()
            or tracking_margin<0 or extra_vertical_buffer<=0):
        raise ValueError('nonnegative margin and positive height buffer required')
    groups=manifest['geometry']['groups']
    home=np.asarray(settings['launch_xy_m'],float);pad=np.asarray(settings['pad_center_xy_m'],float)
    hover=float(settings['takeoff_agl_m']);radius=float(contract['collision_radius_m'])
    half=float(contract['collision_half_height_m'])
    if (home.shape!=(2,) or pad.shape!=(2,) or not np.isfinite([*home,*pad,hover,radius,half]).all()
            or min(hover,radius,half)<=0 or np.linalg.norm(home-pad)<1e-6):
        raise ValueError('finite distinct published home/pad and positive envelope/hover required')
    for point,name in ((home,'Launch mark'),(pad,'Landing pad 5 ft')):
        marker=groups[name];centre=(np.array(marker['min'])+np.array(marker['max']))/2
        if np.linalg.norm(point-centre[:2])>.01:
            raise ValueError('configured home/pad differs from public marker by more than1cm')
    bounds={name:(np.asarray(value['min'],float),np.asarray(value['max'],float)) for name,value in groups.items()}
    if any(a.shape!=(3,) or b.shape!=(3,) or not np.isfinite([*a,*b]).all() or np.any(a>b) for a,b in bounds.values()):
        raise ValueError('finite ordered public geometry bounds required')
    obstruction_top=max(high[2] for name,(low,high) in bounds.items() if not name.lower().startswith('net'))
    roof_bottom=float(bounds['Net roof'][0][2])
    transit=ceil((obstruction_top+half+tracking_margin+extra_vertical_buffer)/.05)*.05
    if transit+half+tracking_margin>=roof_bottom:
        raise ValueError('no vertically separated return band below net roof')
    if hover>=transit or hover-half-tracking_margin<=bounds['Floor 20 x 20 ft'][1][2]:
        raise ValueError('hover must lie above floor envelope and below transit band')
    inner=np.array([bounds['Net side west'][1][0],bounds['Net side east'][0][0],
                    bounds['Net side south'][1][1],bounds['Net side north'][0][1]])
    inflated=radius+tracking_margin
    for point in (pad,home):
        if not (inner[0]+inflated<point[0]<inner[1]-inflated and inner[2]+inflated<point[1]<inner[3]-inflated):
            raise ValueError('return endpoint envelope too close to net')
    waypoints=np.array([[*pad,hover],[*pad,transit],[*home,transit],[*home,hover]])
    metadata=dict(highest_non_net_geometry_z_m=float(obstruction_top),net_roof_bottom_z_m=roof_bottom,
        transit_altitude_m=float(transit),tracking_margin_m=tracking_margin,extra_vertical_buffer_m=extra_vertical_buffer,
        transit_bottom_with_margin_m=float(transit-half-tracking_margin),
        transit_top_with_margin_m=float(transit+half+tracking_margin),
        endpoint_coordinates_source='mission settings checked within1cm of published launch/pad markers',
        vertical_columns_independently_screened=False)
    return waypoints,metadata


def sample_return(points,*,horizontal_budget=None,vertical_budget=None):
    horizontal_budget=horizontal_budget or MotionBudget()
    vertical_budget=vertical_budget or MotionBudget(cruise_mps=.25)
    if not isinstance(horizontal_budget,MotionBudget) or not isinstance(vertical_budget,MotionBudget):
        raise ValueError('validated horizontal/vertical motion budgets required')
    if abs(horizontal_budget.fixed_yaw_rad-vertical_budget.fixed_yaw_rad)>1e-10:
        raise ValueError('this candidate uses one fixed unqualified camera yaw')
    points=np.asarray(points,float)
    if points.shape!=(4,3) or not np.isfinite(points).all():raise ValueError('four finite return waypoints required')
    deltas=np.diff(points,axis=0)
    if (not np.allclose(deltas[[0,2],:2],0) or deltas[0,2]<=0 or deltas[2,2]>=0
            or abs(deltas[1,2])>1e-10 or np.linalg.norm(deltas[1,:2])<=1e-6):
        raise ValueError('requires climb, horizontal translation, descent in order')
    now=0.;rows=[];legs=[];yaw=horizontal_budget.fixed_yaw_rad
    quaternion=[cos(yaw/2),0.,0.,sin(yaw/2)]
    rows.append([0.,*points[0],0.,0.,0.,*quaternion])
    for index,(a,delta) in enumerate(zip(points[:-1],deltas)):
        budget=horizontal_budget if index==1 else vertical_budget
        length=float(np.linalg.norm(delta))
        duration=max(1.875*length/budget.cruise_mps,sqrt((10/np.sqrt(3))*length/budget.acceleration_mps2),
                     (60*length/budget.jerk_mps3)**(1/3))
        count=ceil(duration/budget.dt_s);start=now
        for i in range(1,count+1):
            u=i/count;s=10*u**3-15*u**4+6*u**5
            ds=(30*u*u-60*u**3+30*u**4)/duration
            now=start+u*duration;rows.append([now,*(a+s*delta),*(ds*delta),*quaternion])
        dwell_count=ceil(budget.dwell_s/budget.dt_s)
        for _ in range(dwell_count):
            now+=budget.dwell_s/dwell_count;rows.append([now,*(a+delta),0.,0.,0.,*quaternion])
        legs.append(dict(kind=('climb','over_barrier_translation','descent')[index],length_m=length,
            movement_s=duration,dwell_s=budget.dwell_s,peak_speed_mps=1.875*length/duration,
            peak_acceleration_mps2=(10/np.sqrt(3))*length/duration**2,peak_jerk_mps3=60*length/duration**3,
            budget=asdict(budget)))
    return np.asarray(rows),legs


def build(settings_path,manifest_path,contract_path,output):
    settings=json.loads(settings_path.read_text());manifest=json.loads(manifest_path.read_text());contract=json.loads(contract_path.read_text())
    points,geometry=return_waypoints(settings,manifest,contract)
    budget=MotionBudget(fixed_yaw_rad=float(settings['launch_heading_course_rad']))
    rows,legs=sample_return(points,horizontal_budget=budget,vertical_budget=MotionBudget(cruise_mps=.25,fixed_yaw_rad=budget.fixed_yaw_rad))
    output.mkdir(parents=True,exist_ok=False)
    np.savetxt(output/'trajectory.csv',rows,delimiter=',',header='t,x,y,z,vx,vy,vz,qw,qx,qy,qz',comments='')
    report=dict(kind='offline_optional_return_geometry_not_flight_or_bonus_qualification',rule_scope='supplied rule7.10',
        input_sha256={str(p):sha256(p.read_bytes()).hexdigest() for p in (settings_path,manifest_path,contract_path)},
        published_obj_sha256=manifest['obj_sha256'],generator_sha256=sha256(Path(__file__).read_bytes()).hexdigest(),
        waypoints_xyz_m=points.tolist(),geometry=geometry,legs=legs,reference_duration_s=float(rows[-1,0]),
        scheduling_assumptions_s=dict(previous_complete_return_reserve=40.,
            remaining_inside_that_reserve_for_rearm_takeoff_landing_verification=40.-float(rows[-1,0]),
            current_fresh_idle_and_disarmed_offboard_preflight=60.,
            minimum_with_fresh_ground_qualification_excluding_actual_takeoff_landing=60.+float(rows[-1,0]),
            forty_second_total_reserve_feasible_with_fresh_ground_qualification=False),
        outbound_verification_required=True,flight_completed=False,bonus_qualified=False,finish_time_s=None,
        limitations='Airborne hover-to-hover reference only. Independent mesh screen still required for ascent/descent columns. Native mission currently requires30s idle health plus30s disarmed Offboard preflight for an execution; retaining qualification across outbound landing/rearm has no policy yet. A fresh60s return qualification makes the40s complete-return reserve infeasible. No ROS, rearm, takeoff, landing, timed mission, official referee grade, estimator qualification, visibility guarantee or hardware dynamics.')
    (output/'plan.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    for path,name in ((settings_path,'source-settings.json'),(manifest_path,'public-scene-manifest.json'),(contract_path,'airframe-contract.json')):
        shutil.copy2(path,output/name)
    shutil.copy2(Path(__file__),output/'generator.py')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('settings','public-manifest','contract','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();report=build(args.settings,args.public_manifest,args.contract,args.output)
    print(json.dumps({k:v for k,v in report.items() if k!='legs'},indent=2))
