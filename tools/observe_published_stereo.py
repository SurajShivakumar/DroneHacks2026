"""Live read-only published-map stereo shadow. Logs files; publishes NO topics.

Requires an explicit reviewed static-init world/VIO alignment, calibrated
rectified stereo and a downward single-ray LaserScan. This diagnostic does
not grant readiness, arm, or replace the existing VIO guard. Optional XY fusion
is logged to files only, with no odometry output.
"""
import argparse
from copy import copy
from dataclasses import asdict
from collections import Counter,deque
from hashlib import sha256
import json
from pathlib import Path
import shutil
import signal
import sys
import time
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dronebench.stereo_observer import CausalSamples,decode_mono8,body_height_from_range
from dronebench.depth_mapping import Pinhole
from dronebench.published_stereo import PublishedSparseStereo
from dronebench.published_raycast import build_published_raycast
from dronebench.published_wall_map import build_published_wall_map
from dronebench.depth_wall_constraint import extract_wall_constraints
from dronebench.vision_odometry import OdometrySample
from dronebench.map_vio import MapVioCalibration,DeclaredHeadingAlignment
from dronebench.wall_kinematic_filter import WallKinematicFilter
from tools.ros_depth_mapper import checked_camera,stamp_ns,require_fresh


def yaw_matrix(angle):
    c,s=np.cos(angle),np.sin(angle)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])


def checked_stereo_camera(info,image,frame,*,baseline_m):
    """Validate each rectified eye, including standard right-eye P[0,3]=-fx*b.

    The depth mapper correctly requires zero translation for a depth image's
    own optical frame. Stereo CameraInfo has a different projection contract.
    Preserve that stricter depth checker; validate this known offset here.
    """
    p=np.asarray(info.p,float).reshape(3,4)
    if not np.isfinite(baseline_m) or baseline_m<0 or not np.allclose(p[:,3],[-p[0,0]*baseline_m,0,0],atol=1e-5,rtol=0):
        raise ValueError('stereo projection baseline differs from calibrated extrinsics')
    normalized=copy(info); normalized.p=p.copy().ravel().tolist()
    for index in (3,7,11):normalized.p[index]=0.
    return checked_camera(normalized,image,frame,.1)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('repo','calibration','airframe-contract','initialization','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=150)
    parser.add_argument('--use-sim-time',action='store_true')
    parser.add_argument('--fuse-shadow',action='store_true',help='experimental causal XY/bias filter in file log ONLY; no odometry or flight output')
    args=parser.parse_args()
    if not 30<=args.seconds<=600:parser.error('duration must be 30..600 wall seconds')
    init=json.loads(args.initialization.read_text())
    if init['scope']!='declared_static_init_diagnostic_not_flight':
        raise ValueError('reviewed diagnostic initialization required; no automatic reanchoring')
    p0=np.asarray(init['initial_body_position_map_m'],float)
    if p0.shape!=(3,) or not np.isfinite(p0).all():raise ValueError('finite declared launch mark required')
    launch=yaw_matrix(init['initial_body_yaw_map_rad'])
    error=np.asarray(init['prior_error_xyz_m'],float)
    angular_error=float(init['angular_error_rad'])
    floor_z=float(init['floor_z_m'])
    if error.shape!=(3,) or not np.isfinite([*error,angular_error,floor_z]).all() or min(error)<=0 or angular_error<=0:
        raise ValueError('finite positive conservative diagnostic prior bounds required')
    contract=json.loads(args.airframe_contract.read_text()); T=np.asarray(contract['body_from_imu'],float)
    chain=yaml.safe_load(args.calibration.read_text().replace('%YAML:1.0',''))
    a,b=chain['cam0'],chain['cam1']; fx,fy,cx,cy=a['intrinsics']; width,height=a['resolution']
    if a['intrinsics']!=b['intrinsics'] or any(abs(x)>1e-10 for x in a['distortion_coeffs']+b['distortion_coeffs']):
        raise ValueError('equal zero-distortion rectified calibration required')
    sys.path.insert(0,str(args.repo.resolve()))
    from drl.config import load_config
    from drl.course import load_obj_groups
    config,course,_=load_config(str(args.repo.resolve()/'config/course_v3.yaml'))
    obj=args.repo/config['course_obj']; _,groups=load_obj_groups(str(obj))
    net=np.concatenate([points for name,points in groups.items() if name.startswith('Net')])
    bounds=tuple(v for axis in range(3) for v in (net[:,axis].min(),net[:,axis].max()))
    geometry=build_published_raycast(course,published_provenance='published OBJ:'+sha256(obj.read_bytes()).hexdigest(),net_bounds=bounds,floor_z=floor_z)
    sensor=PublishedSparseStereo(geometry,Pinhole(width,height,fx,fy,cx,cy),T@a['T_imu_cam'],T@b['T_imu_cam'])
    frd=np.diag([1.,-1.,-1.]); offset=frd@contract['range_offset_frd_m']; axis=frd@contract['range_axis_frd']
    origin_imu=p0+launch@T[:3,3]
    calibration=None
    heading=DeclaredHeadingAlignment(T,init['initial_body_yaw_map_rad'],init['heading_valid_source_window_s'])
    heading_processed=0.
    args.output.mkdir(parents=True,exist_ok=False)
    source=args.output/'source'; source.mkdir()
    hashes={}
    for folder in ('dronebench','tools'):
        for file in (Path(__file__).resolve().parents[1]/folder).glob('*.py'):
            target=source/folder/file.name; target.parent.mkdir(exist_ok=True); shutil.copy2(file,target)
            hashes[str(Path(folder)/file.name)]=sha256(target.read_bytes()).hexdigest()
    for name,path in (('calibration.yaml',args.calibration),('airframe.json',args.airframe_contract),('initialization.json',args.initialization)):
        shutil.copy2(path,args.output/name)
    (args.output/'source-hashes.json').write_text(json.dumps(hashes,indent=2))
    (args.output/'arguments.json').write_text(json.dumps({k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},indent=2))
    (args.output/'published-map.json').write_text(json.dumps(dict(obj_sha256=sha256(obj.read_bytes()).hexdigest(),net_bounds=list(bounds),floor_z_m=floor_z),indent=2))
    loader=source/'published_loader';loader.mkdir()
    for relative in ('drl/config.py','drl/course.py','config/course_v3.yaml'):
        file=args.repo/relative
        shutil.copy2(file,loader/file.name)
    import cv2
    cv2.setNumThreads(1)
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image,CameraInfo,LaserScan
    from nav_msgs.msg import Odometry
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,signal.default_int_handler)
    signal.signal(signal.SIGTERM,signal.default_int_handler)
    node=rclpy.create_node('published_stereo_read_only_shadow')
    node.set_parameters([Parameter('use_sim_time',value=args.use_sim_time)])
    qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
    histories={name:CausalSamples() for name in ('vio','range','left_info','right_info')}
    images={'left':deque(maxlen=5),'right':deque(maxlen=5)}
    rejects=Counter(); counts=Counter(); records=[]; subscriptions=[]; last_source=0; fatal=None
    log=(args.output/'observations.jsonl').open('x')
    updates=(args.output/'filter-updates.jsonl').open('x')
    def on_history(name,msg):
        nonlocal fatal
        counts[name]+=1
        try:histories[name].append(stamp_ns(msg.header.stamp)/1e9,msg,time.monotonic())
        except ValueError as exc:fatal=str(exc)
    def on_image(name,msg):
        counts[name]+=1; images[name].append((stamp_ns(msg.header.stamp)/1e9,msg,time.monotonic()))
    for name,topic,cls in (('vio','/vio_shadow/odomimu',Odometry),('range','/range/scan',LaserScan),
                          ('left_info',a['rostopic'].replace('/image','/camera_info'),CameraInfo),
                          ('right_info',b['rostopic'].replace('/image','/camera_info'),CameraInfo)):
        subscriptions.append(node.create_subscription(cls,topic,lambda msg,n=name:on_history(n,msg),qos))
    for name,topic in (('left',a['rostopic']),('right',b['rostopic'])):
        subscriptions.append(node.create_subscription(Image,topic,lambda msg,n=name:on_image(n,msg),qos))
    started=time.monotonic(); next_status=started+10
    state=None; filter_time=None; processed_vio=0.; last_update=None; shadow_fault=None; stopped_early=False
    try:
        while time.monotonic()-started<args.seconds:
            rclpy.spin_once(node,timeout_sec=.005)
            if time.monotonic()>=next_status:
                print(json.dumps(dict(elapsed_wall_s=time.monotonic()-started,counts=dict(counts),accepted=len(records),rejections=dict(rejects))),flush=True)
                next_status=time.monotonic()+10
            if fatal:raise ValueError(fatal)
            if shadow_fault:raise ValueError(shadow_fault)
            if not images['left'] or not images['right']:continue
            left_t,left,left_arrival=images['left'][0]
            right_t,right,right_arrival=min(images['right'],key=lambda row:abs(row[0]-left_t))
            now=time.monotonic()
            if abs(left_t-right_t)>.01:
                if right_t>left_t or now-left_arrival>.5:
                    images['left'].popleft(); rejects['unpaired_stereo']+=1
                continue
            # Wait briefly for causal VIO/info/range callbacks; never select future samples.
            try:
                sampled={name:h.at(left_t,now,max_skew=(.05 if name=='vio' else .1)) for name,h in histories.items()}
            except ValueError:
                if now-left_arrival<.1:continue
                images['left'].popleft(); rejects['missing_causal_support']+=1; continue
            images['left'].popleft()
            if left_t<=last_source:raise ValueError('image source reset/nonadvancing; restart with reviewed initialization')
            last_source=left_t; begin=time.monotonic()
            try:
                require_fresh(int(left_t*1e9),node.get_clock().now().nanoseconds,.5,'stereo')
                if max(now-left_arrival,now-right_arrival)>.5:raise ValueError('stale image arrival')
                for side,img,cal in (('left',left,a),('right',right,b)):
                    info=sampled[side+'_info'][1]
                    camera=checked_stereo_camera(info,img,init[side+'_frame'],baseline_m=(0. if side=='left' else sensor.config.baseline_m))
                    if ((camera.width,camera.height)!=(sensor.camera.width,sensor.camera.height) or
                            not np.allclose([camera.fx,camera.fy,camera.cx,camera.cy],[fx,fy,cx,cy],atol=1e-4,rtol=0)):
                        raise ValueError('live CameraInfo differs from calibration file')
                vio=sampled['vio'][1]
                if vio.header.frame_id!='global' or vio.child_frame_id!='imu':raise ValueError('VIO frame contract changed')
                if calibration is None:
                    try:
                        for t,msg,_ in histories['vio'].rows:
                            if t>left_t:break
                            if t<=heading_processed:continue
                            heading_processed=t
                            aligned=heading.observe(OdometrySample.from_ros(msg,clock_domain='ROS node clock'))
                            if aligned is not None:
                                calibration=MapVioCalibration(T,aligned,origin_imu)
                                (args.output/'alignment.json').write_text(json.dumps(dict(
                                    kind='declared_heading_diagnostic_only_not_flight_qualification',
                                    map_from_vio_rotation=aligned.tolist(),declared_heading_rad=init['initial_body_yaw_map_rad'],
                                    first_source_s=heading.samples[0][0],last_source_s=heading.samples[-1][0],samples=len(heading.samples),
                                    translation_reanchored=False,truth_used=False),indent=2))
                                break
                    except ValueError as exc:
                        shadow_fault='diagnostic heading initialization rejected: '+str(exc)
                        raise
                    if calibration is None:raise ValueError('waiting for stable declared-heading calibration')
                observation=calibration.body(OdometrySample.from_ros(vio,clock_domain='ROS node clock'))
                R=observation.rotation; position=observation.position.copy()
                if args.fuse_shadow:
                    try:
                        for t,msg,_ in histories['vio'].rows:
                            if t>left_t:break
                            if t<=processed_vio:continue
                            processed_vio=t
                            if filter_time is not None and t<filter_time:continue
                            item=calibration.body(OdometrySample.from_ros(msg,clock_domain='ROS node clock'))
                            if state is None:
                                state=WallKinematicFilter(item.position[:2],item.velocity[:2]); filter_time=last_update=t
                            else:
                                if t>filter_time:state.predict(t-filter_time)
                                filter_time=t
                                if t-last_update>=.1:
                                    state.observe_vio_position(item.position[:2],item.position_covariance[:2,:2])
                                    state.observe_velocity(item.velocity[:2],item.velocity_covariance[:2,:2]); last_update=t
                                    updates.write(json.dumps(dict(source_s=t,observations={
                                        k:state.last_observations[k] for k in ('vio_position','vio_velocity')}),allow_nan=False)+'\n');updates.flush()
                        if state is None:raise ValueError('no causal VIO to initialize shadow')
                        if left_t>filter_time:state.predict(left_t-filter_time); filter_time=left_t
                        position[:2]=state.position
                    except ValueError as exc:
                        shadow_fault='shadow continuity lost: '+str(exc)
                        raise
                scan=sampled['range'][1]
                if scan.header.frame_id!=init['range_frame']:raise ValueError('range calibrated frame changed')
                if len(scan.ranges)!=1 or abs(scan.angle_min)>1e-6 or abs(scan.angle_max)>1e-6:
                    raise ValueError('requires calibrated single downward range ray')
                distance=float(scan.ranges[0])
                if not scan.range_min<=distance<=scan.range_max:raise ValueError('range outside calibrated bounds')
                height=body_height_from_range(distance,R,offset,axis); position[2]=floor_z+height
                prior_error=error.copy()
                if args.fuse_shadow:prior_error[:2]=3*np.sqrt(np.diag(state.covariance[:2,:2]))
                measured=sensor.observe(decode_mono8(left),decode_mono8(right),position=position,rotation=R,error_xyz=prior_error,angular_error_rad=angular_error)
                yaw=float(np.arctan2(R[1,0],R[0,0])); points=measured.points_base_xyz@(yaw_matrix(-yaw)@R).T
                points=points[(points[:,2]+position[2]>.3)&(points[:,2]+position[2]<1.5)]
                walls=build_published_wall_map(course,z=position[2],net_bounds=bounds[:4])
                factors=extract_wall_constraints(points[:,:2],walls.segments,(*position[:2],yaw))
                processing=time.monotonic()-begin
                require_fresh(int(left_t*1e9),node.get_clock().now().nanoseconds,.5,'processed stereo')
                if processing>.1:raise ValueError('processing exceeded 100ms wall bound')
                fused=0
                if args.fuse_shadow:
                    for factor in factors.constraints:
                        if factor.accepted and state.observe_wall(factor.normal,factor.projection,factor.variance):fused+=1
                row=dict(source_s=left_t,right_source_s=right_t,support_source_s={k:v[0] for k,v in sampled.items()},
                    position_prior_map_m=position.tolist(),measured_height_m=height,processing_wall_s=processing,
                    raw_vio_xy_m=observation.position[:2].tolist(),
                    sensor=measured.diagnostics,accepted_wall_factors=sum(f.accepted for f in factors.constraints),
                    factors=[asdict(f) for f in factors.constraints],rotation_map_from_body=R.tolist(),prior_error_xyz_m=prior_error.tolist(),
                    filtered_xy_m=state.position.tolist() if args.fuse_shadow else None,
                    filtered_velocity_xy_mps=state.velocity.tolist() if args.fuse_shadow else None,
                    covariance_xy_m2=state.covariance[:2,:2].tolist() if args.fuse_shadow else None,wall_factors_fused=fused)
                log.write(json.dumps(row,allow_nan=False)+'\n'); log.flush(); records.append(row)
            except ValueError as exc:rejects[str(exc)]+=1
    except ValueError as exc:
        fatal=str(exc)
    except KeyboardInterrupt:
        stopped_early=True
    finally:
        log.close();updates.close()
        report=dict(kind='live_read_only_stereo_diagnostic_not_estimator_or_flight',published_topics=False,
            duration_wall_s=time.monotonic()-started,counts=dict(counts),accepted_frames=len(records),rejections=dict(rejects),fatal=fatal,
            opencv_version=cv2.__version__,numpy_version=np.__version__,
            shadow_fusion=args.fuse_shadow,
            stopped_by_harness_or_operator=stopped_early,
            median_processing_wall_s=float(np.median([r['processing_wall_s'] for r in records])) if records else None,
            limitations='Declared diagnostic heading/start placement and engineering uncertainty assumptions. No reset recovery, flight-readiness, odometry output, or independent pose accuracy claim.')
        (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)); print(json.dumps(report,indent=2))
        node.destroy_node(); rclpy.try_shutdown()
    return 1 if fatal or not records else 0


if __name__=='__main__':raise SystemExit(main())
