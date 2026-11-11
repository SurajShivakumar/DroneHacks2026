"""Offline VIO-versus-scene comparison; never imports scene truth into flight.

This aligns VIO and Gazebo once at the first shared source timestamp, then
reports drift over the captured interval. Use only for diagnostic evidence.
"""
import argparse
from hashlib import sha256
import json
from pathlib import Path

import numpy as np


def matrix_wxyz(q):
    q=np.asarray(q,float)
    if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.001:
        raise ValueError('finite unit wxyz quaternion required')
    w,x,y,z=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def scene_pose(times,states,t):
    i=int(np.searchsorted(times,t,side='right')-1)
    i=max(0,min(i,len(times)-2))
    u=float(np.clip((t-times[i])/(times[i+1]-times[i]),0,1))
    p=(1-u)*states[i,1:4]+u*states[i+1,1:4]
    q0=states[i,4:8].copy();q1=states[i+1,4:8].copy()
    if np.dot(q0,q1)<0:q1=-q1
    q=(1-u)*q0+u*q1;q/=np.linalg.norm(q)
    return p,matrix_wxyz(q)


def compare(poses_path,odometry_path,contract_path,output_path):
    poses=np.genfromtxt(poses_path,delimiter=',',names=True)
    times=poses['t'];states=np.stack([poses[k] for k in ('t','x','y','z','qw','qx','qy','qz')],axis=-1)
    if len(states)<2 or not np.isfinite(states).all() or np.any(np.diff(times)<=0):
        raise ValueError('monotonic finite independent scene pose samples required')
    contract=json.loads(contract_path.read_text())
    transform=np.asarray(contract['body_from_imu'],float)
    if transform.shape!=(4,4) or not np.isfinite(transform).all():
        raise ValueError('calibrated body_from_imu transform required')
    imu_in_body=transform[:3,3]
    rows=[]
    for line in odometry_path.read_text(encoding='utf-8').splitlines():
        item=json.loads(line)
        if item.get('topic')!='/vio_shadow/odomimu' or item.get('frame_id')!='global' or item.get('child_frame_id')!='imu':
            raise ValueError('recorded OpenVINS global/imu odometry contract changed')
        t=float(item['source_s']); p=np.array([item['position'][k] for k in ('x','y','z')],float)
        q=np.array([item['orientation'][k] for k in ('w','x','y','z')],float)
        if not np.isfinite([t,*p,*q]).all() or abs(np.linalg.norm(q)-1)>.001:
            raise ValueError('finite source-stamped odometry with unit quaternion required')
        if times[0]<=t<=times[-1]:rows.append((t,p,matrix_wxyz(q)))
    if len(rows)<2:raise ValueError('at least two overlapping VIO and scene samples required')
    truth0,R_world_body0=scene_pose(times,states,rows[0][0])
    truth_imu0=truth0+R_world_body0@imu_in_body
    R_world_imu0=R_world_body0@transform[:3,:3]
    R_align=R_world_imu0@rows[0][2].T
    errors=[]
    attitude_errors=[]
    for t,p_vio,R_vio in rows:
        truth_body,R_world_body=scene_pose(times,states,t)
        truth_imu=truth_body+R_world_body@imu_in_body
        aligned=truth_imu0+R_align@(p_vio-rows[0][1])
        error=aligned-truth_imu
        errors.append((t,error,aligned,truth_imu))
        expected_rotation=R_world_body@transform[:3,:3]
        rotation_error=expected_rotation.T@R_align@R_vio
        attitude_errors.append(float(np.arccos(np.clip((np.trace(rotation_error)-1)/2,-1.,1.))))
    vectors=np.stack([r[1] for r in errors]);norms=np.linalg.norm(vectors,axis=1)
    worst=int(np.argmax(norms));rmse=float(np.sqrt(np.mean(norms**2)))
    summary={'kind':'offline_scene_truth_comparison_never_navigation_input',
        'input_sha256':{str(p):sha256(p.read_bytes()).hexdigest() for p in (poses_path,odometry_path,contract_path)},
        'samples':len(errors),'source_interval_s':[float(errors[0][0]),float(errors[-1][0])],
        'initial_alignment':'one constant rotation and translation from first overlapping pose; no trajectory fit',
        'body_from_imu_translation_m':imu_in_body.tolist(),
        'position_error_rmse_m':rmse,'maximum_position_error_m':float(norms[worst]),
        'maximum_error_source_s':float(errors[worst][0]),
        'maximum_error_vector_m':vectors[worst].tolist(),
        'final_error_vector_m':vectors[-1].tolist(),
        'attitude_error_rmse_rad':float(np.sqrt(np.mean(np.square(attitude_errors)))),
        'maximum_attitude_error_rad':float(max(attitude_errors)),
        'maximum_attitude_error_source_s':float(errors[int(np.argmax(attitude_errors))][0]),
        'limitations':'Offline diagnostic comparison only. Scene poses are withheld from ROS navigation. Does not qualify hardware, flight safety, estimator quality, or course completion.'}
    with output_path.open('x',encoding='utf-8') as stream:json.dump(summary,stream,indent=2)
    print(json.dumps(summary,indent=2));return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--poses',type=Path,required=True)
    parser.add_argument('--odometry',type=Path,required=True)
    parser.add_argument('--contract',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();compare(args.poses,args.odometry,args.contract,args.output)
