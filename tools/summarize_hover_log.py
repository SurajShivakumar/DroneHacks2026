"""Offline native ULog hover measurements, never a course or hardware grade."""
import argparse
from hashlib import sha256
import json
from pathlib import Path

import numpy as np


def summarize(ulog_path, mission_path):
    from pyulog import ULog
    mission=json.loads(mission_path.read_text())
    log=ULog(str(ulog_path),message_name_filter_list=[
        'vehicle_status','vehicle_local_position','vehicle_land_detected','failure_detector_status'])
    status=log.get_dataset('vehicle_status').data
    armed=np.flatnonzero(status['arming_state']==2)  # pinned PX4 v1.16 enum
    result={'kind':'native_estimator_hover_measurements_not_course_grade',
        'ulog_sha256':sha256(ulog_path.read_bytes()).hexdigest(),
        'mission_sha256':sha256(mission_path.read_bytes()).hexdigest(),
        'mission_state':mission['state'],'course_finish_time_s':None,
        'hardware_qualified':False,'armed_observed_in_native_log':bool(len(armed))}
    if not len(armed):
        return result
    start=int(armed[0]); after=np.flatnonzero((np.arange(len(status['timestamp']))>armed[-1]) & (status['arming_state']!=2))
    begin=int(status['timestamp'][start]); end=int(status['timestamp'][after[0]]) if len(after) else None
    result.update(armed_source_us=begin,disarmed_source_us=end,
        arm_to_disarm_sim_s=(end-begin)/1e6 if end is not None else None)
    position=log.get_dataset('vehicle_local_position').data
    times=position['timestamp']; first=max(0,int(np.searchsorted(times,begin))-1)
    mask=(times>=begin)&(times<=(end if end is not None else times[-1]))
    if np.any(mask):
        rise=position['z'][first]-position['z'][mask]
        xy=np.hypot(position['x'][mask]-position['x'][first],position['y'][mask]-position['y'][first])
        yaw=position['heading'][mask]-position['heading'][first]
        result.update(max_estimated_rise_m=float(np.max(rise)),
            max_estimated_xy_displacement_m=float(np.max(xy)),
            max_estimated_heading_change_rad=float(np.max(np.abs(np.arctan2(np.sin(yaw),np.cos(yaw))))))
    land=log.get_dataset('vehicle_land_detected').data
    air=np.flatnonzero((land['timestamp']>=begin)&(land['timestamp']<=(end or land['timestamp'][-1]))&(~land['landed'].astype(bool)))
    result['airborne_observed_in_native_log']=bool(len(air))
    if len(air):
        touchdown=np.flatnonzero((land['timestamp']>land['timestamp'][air[-1]])&land['landed'].astype(bool))
        result['takeoff_detected_source_us']=int(land['timestamp'][air[0]])
        result['touchdown_detected_source_us']=int(land['timestamp'][touchdown[0]]) if len(touchdown) else None
    faults=log.get_dataset('failure_detector_status').data
    mask=(faults['timestamp']>=begin)&(faults['timestamp']<=(end or faults['timestamp'][-1]))
    result['native_esc_failure_observed']=bool(np.any(faults['fd_arm_escs'][mask]))
    result['limitations']='Displacement uses native onboard estimates, not independent simulator truth. No obstacle, checkpoint, pad footprint or course completion evaluation.'
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ulog',type=Path,required=True)
    parser.add_argument('--mission',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=summarize(args.ulog,args.mission)
    with args.output.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2))
