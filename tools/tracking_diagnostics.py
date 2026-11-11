"""Reconstruct tracking errors from recorded native odometry, never control I/O."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import numpy as np


def inspect(journal):
    frame=None; odometry=[]; records=[]
    for line in journal.read_text().splitlines():
        item=json.loads(line)
        if item['kind']=='frozen_launch_alignment':frame=item
        elif item['kind']=='telemetry' and item.get('stream')=='odom':
            odometry.append([item['source_s'],*item['fields']['position']])
        elif item['kind']=='reference_tracking':records.append(item)
    if not frame or not odometry or not records:raise ValueError('complete recorded tracking and frame evidence required')
    north=frame['north_heading_course']; c,s=np.cos(north),np.sin(north)
    matrix=np.array([[c,s,0],[s,-c,0],[0,0,-1]])
    samples=np.asarray(odometry)
    measured=samples[:,1:]@matrix.T+np.array(frame['origin_course'])
    rows=[]
    for item in records:
        index=max(0,int(np.searchsorted(samples[:,0],item['source_sim_s'],side='right'))-1)
        error=measured[index]-np.array(item['target_course_m'])
        rows.append(dict(source_sim_s=item['source_sim_s'],reference_elapsed_s=item['reference_elapsed_s'],
            measured_course_m=measured[index].tolist(),target_course_m=item['target_course_m'],
            measured_minus_target_m=error.tolist(),reported_error_m=item['error_m'],
            velocity_feedforward_ned_mps=item['velocity_feedforward_ned_mps'],
            speed_fraction=item.get('speed_fraction')))
    return {'kind':'offline_reconstruction_from_recorded_causal_native_odometry_not_truth',
        'journal_sha256':sha256(journal.read_bytes()).hexdigest(),'samples':len(rows),
        'maximum_reported_error_m':max(r['reported_error_m'] for r in rows),
        'maximum_absolute_reconstructed_axis_error_m':np.max(np.abs([r['measured_minus_target_m'] for r in rows]),axis=0).tolist(),
        'last_samples':rows[-10:]}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--journal',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=inspect(args.journal)
    with args.output.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result,indent=2))
