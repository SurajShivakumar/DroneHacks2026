"""Sky Beta mission adapter. Default read-only; never grants flight approval.

Uses actual ROS/PX4 telemetry, the teammate inference core and saved trajectory.
Execute requires a completed airframe contract and recorded team approval.
Return execution stays blocked until the outbound run is independently graded;
the supervisor's return sequence is implemented and tested separately.
"""
import argparse
from dataclasses import asdict, replace
from hashlib import sha256
import json
from math import atan2, nan
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dronebench.causal_telemetry import CausalInbox
from dronebench.course_mission import CourseMission, MissionPlan
from dronebench.disarmed_offboard import native_commander_ack_matches
from dronebench.teammate_policy import CourseFrame, ReferenceTrajectory, TeammatePolicy, TrackingClock, TrackingDeviation, floor_height_from_range, rotation_wxyz
from dronebench.telemetry import json_safe
from dronebench.vio_quality import VioFeatureMonitor, finite_xyz_count
from tools.hover_qualification import candidate_topics


ACCEL_FF_LIMIT = .5      # m/s^2: references are generated at <=0.16
YAWSPEED_FF_LIMIT = .5   # rad/s: matches TrackingClock's yaw-rate ceiling


def bounded_feedforward(vector, limit):
    """Finite 3-vector, norm-clipped to `limit` (never amplifies)."""
    vector = np.asarray(vector, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError('finite NED acceleration feedforward required')
    norm = float(np.linalg.norm(vector))
    return vector*(limit/norm) if norm > limit else vector


def reference_rates(reference, elapsed_s, h=.05):
    """Central-difference reference acceleration (course frame, per reference
    second^2) and course-yaw rate (per reference second) at elapsed_s."""
    lo, hi = max(0., elapsed_s-h), min(reference.duration_s, elapsed_s+h)
    if hi-lo < 1e-6:
        return np.zeros(3), 0.
    _, v_lo = reference.sample(lo)
    _, v_hi = reference.sample(hi)
    return (v_hi-v_lo)/(hi-lo), (reference.yaw(hi)-reference.yaw(lo))/(hi-lo)


def validate_execution_contract(contract):
    if (contract.get('flight_model_ready') is not True or contract.get('missing')
            or contract.get('motor_plugins_enabled') is not True):
        raise ValueError('Sky Beta actuator/propeller model is incomplete; see airframe contract')
    for key in ('collision_radius_m', 'collision_half_height_m'):
        value = contract.get(key)
        if type(value) not in (int, float) or not np.isfinite(value) or value <= 0:
            raise ValueError('complete reviewed airframe envelope required')


def validate_bounded_qualification(settings, plan, duration_s):
    if settings.get('bounded_navigation_qualification') is not True:
        raise ValueError('explicit bounded navigation qualification required')
    if (plan.task != 'trajectory' or plan.return_enabled or plan.max_leg_sim_s>90
            or not np.isfinite(duration_s) or not 0<duration_s<=60):
        raise ValueError('qualification requires trajectory <=60s, leg <=90s and no return')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--settings', type=Path, required=True)
    parser.add_argument('--airframe-contract', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--approval-reference')
    parser.add_argument('--shared-simulation-clock', action='store_true')
    parser.add_argument('--observe-seconds', type=float, default=40.)
    parser.add_argument('--repo', type=Path)
    parser.add_argument('--policy', type=Path)
    parser.add_argument('--trajectory', type=Path)
    args = parser.parse_args()
    settings = json.loads(args.settings.read_text())
    contract = json.loads(args.airframe_contract.read_text())
    plan = MissionPlan(**settings['mission'])
    bounded_qualification=settings.get('bounded_navigation_qualification',False)
    if type(bounded_qualification) is not bool:
        parser.error('bounded_navigation_qualification must be boolean')
    if bounded_qualification:
        if not args.trajectory:
            parser.error('bounded qualification needs a saved trajectory')
        try:
            validate_bounded_qualification(settings,plan,ReferenceTrajectory(
                args.trajectory,time_scale=settings.get('trajectory_time_scale',4.)).duration_s)
        except ValueError as error:
            parser.error(str(error))
    if args.execute:
        if not args.approval_reference or not args.approval_reference.strip() or not args.shared_simulation_clock:
            parser.error('execute requires recorded team approval-reference and shared-simulation-clock')
        try:
            validate_execution_contract(contract)
        except ValueError as error:
            parser.error(str(error))
        if plan.return_enabled:
            parser.error('first outbound qualification cannot auto-return before independent evaluation')
        if plan.task != 'hover' and (not args.repo or not args.trajectory or not settings.get('launch_placement_reviewed')):
            parser.error('navigation needs pinned source/reference and reviewed physical launch placement')
        if plan.task == 'ppo' and not args.policy:
            parser.error('PPO requires original ONNX checkpoint')
    if not 30 <= args.observe_seconds <= 120:
        parser.error('read-only observation must be 30..120 wall seconds')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as output, args.output.with_suffix('.jsonl').open('x', encoding='utf-8') as journal:
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, VehicleLandDetected, VehicleCommandAck,
            VehicleCommand, VehicleOdometry, HoverThrustEstimate, OffboardControlMode, TrajectorySetpoint, VehicleRatesSetpoint)
        from sensor_msgs.msg import LaserScan, PointCloud2
        rclpy.init()
        node = rclpy.create_node('dronehacks_course_mission')
        node.set_parameters([Parameter('use_sim_time', value=True)])
        started = time.monotonic()
        mission = CourseMission(started_wall=started, plan=plan, execute=args.execute, approval_reference=args.approval_reference)
        vio_quality = VioFeatureMonitor()
        vio_quality_log_state = None
        inbox, subscriptions, topics, publishers, sent = CausalInbox(), {}, {}, {}, {}
        auxiliary, frame, policy, reference = {}, None, None, None
        route_gates, navigation_loaded = None, False
        if args.trajectory:
            reference = ReferenceTrajectory(args.trajectory, time_scale=settings.get('trajectory_time_scale', 4.))
        source_component = 191
        exception = None
        reference_elapsed, last_nav_sim = 0., None
        tracking_clock=TrackingClock(max_error_m=settings.get('tracking_max_error_m',.25),
            pause_error_m=settings.get('tracking_pause_error_m',.12),rate_slew_per_s=settings.get('tracking_rate_slew_per_s'),
            max_yaw_rate=settings.get('max_yaw_rate_rad_s',.5),lag_pause_m=settings.get('tracking_lag_pause_m'))
        route_gate, previous_course = 0, None
        gate_events = []
        qualification_endpoint_observed = False
        types = {'status': VehicleStatus, 'position': VehicleLocalPosition, 'land': VehicleLandDetected,
                 'ack': VehicleCommandAck, 'odom': VehicleOdometry, 'hover': HoverThrustEstimate,
                 'command': VehicleCommand, 'heartbeat': OffboardControlMode,
                 'trajectory': TrajectorySetpoint, 'rates': VehicleRatesSetpoint}
        names = dict(status='vehicle_status', position='vehicle_local_position', land='vehicle_land_detected',
                     ack='vehicle_command_ack', odom='vehicle_odometry', hover='hover_thrust_estimate',
                     command='vehicle_command', heartbeat='offboard_control_mode', trajectory='trajectory_setpoint', rates='vehicle_rates_setpoint')

        def log(value):
            journal.write(json.dumps(json_safe(value), allow_nan=False)+'\n'); journal.flush()

        def callback(name, msg):
            wall = time.monotonic()
            if name == 'vio_slam':
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                try:
                    count = finite_xyz_count(msg)
                    vio_quality.observe(source_s=stamp, received_wall_s=wall, feature_count=count)
                except Exception as error:
                    vio_quality.invalidate('cannot decode OpenVINS SLAM feature cloud: '+str(error), received_wall_s=wall)
                clock_s = node.get_clock().now().nanoseconds/1e9
                # use_sim_time: the clock reads 0 until the first /clock message
                # arrives (v36 startup race). Freshness cannot be judged yet, so
                # wait; nothing can arm or publish before simulation time exists.
                state = (vio_quality.status(now_sim_s=clock_s, now_wall_s=wall) if clock_s > 0
                         else {'state': 'WAITING', 'reason': 'ROS simulation clock not yet received'})
                log({'kind': 'vio_slam_features', 'source_s': stamp,
                     'feature_count': vio_quality.latest_count, 'quality_state': state['state'],
                     'quality_reason': state.get('reason'), 'ros_clock_s': clock_s,
                     'feature_source_age_s': clock_s-stamp, 'arrival_wall_s': wall})
                return
            if name == 'ack':
                log({'kind': 'received_command_ack', 'source_s': msg.timestamp/1e6,
                     'target_system': msg.target_system, 'target_component': msg.target_component,
                     'from_external': msg.from_external, 'command': msg.command, 'result': msg.result})
                kind = sent.get(msg.command)
                if native_commander_ack_matches(msg, expected_commands=sent,
                        target_system=mission._status().get('system_id'), target_component=source_component, request_sent=kind is not None):
                    result = 'accepted' if msg.result == msg.VEHICLE_CMD_RESULT_ACCEPTED else 'in_progress' if msg.result == msg.VEHICLE_CMD_RESULT_IN_PROGRESS else 'rejected'
                    mission.acknowledge(kind, result=result, stamp_s=msg.timestamp/1e6, received_wall=wall)
                return
            stamp = msg.timestamp/1e6 if name not in ('range',) else msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if name == 'status':
                fields = dict(armed=msg.arming_state == msg.ARMING_STATE_ARMED,
                    disarmed=msg.arming_state == msg.ARMING_STATE_DISARMED, offboard=msg.nav_state == msg.NAVIGATION_STATE_OFFBOARD,
                    auto_loiter=msg.nav_state == msg.NAVIGATION_STATE_AUTO_LOITER, land_mode=msg.nav_state == msg.NAVIGATION_STATE_AUTO_LAND,
                    startup_mode_allowed=msg.nav_state == msg.NAVIGATION_STATE_AUTO_LOITER, preflight=msg.pre_flight_checks_pass,
                    failsafe=msg.failsafe, system_id=msg.system_id, component_id=msg.component_id, gcs_connection_lost=msg.gcs_connection_lost)
            elif name == 'position':
                stamp = msg.timestamp_sample/1e6
                fields = {key: getattr(msg, key) for key in ('x','y','z','vx','vy','vz','heading','xy_valid','z_valid','v_xy_valid','v_z_valid','heading_good_for_control')+mission.reset_fields}
            elif name == 'land':
                fields = dict(landed=msg.landed, freefall=msg.freefall, ground_contact=msg.ground_contact)
            elif name == 'range':
                fields = dict(distance=float(msg.ranges[0]) if len(msg.ranges) == 1 else nan,
                              minimum=msg.range_min, maximum=msg.range_max, frame=msg.header.frame_id)
            elif name == 'hover':
                fields = dict(valid=msg.valid, thrust=msg.hover_thrust)
            else:
                stamp = msg.timestamp_sample/1e6
                fields = dict(position=list(msg.position), q=list(msg.q), velocity=list(msg.velocity),
                    angular=list(msg.angular_velocity), pose_frame=msg.pose_frame, velocity_frame=msg.velocity_frame,
                    position_variance=list(msg.position_variance), velocity_variance=list(msg.velocity_variance))
            inbox.put(name, stamp, wall, fields)
            log({'kind': 'telemetry', 'stream': name, 'source_s': stamp, 'arrival_wall': wall, 'fields': fields})

        def discover():
            graph = node.get_topic_names_and_types()
            for name in ('status','position','land','ack','odom','hover'):
                if name in subscriptions:
                    continue
                choices = candidate_topics(graph, direction='out', basename=names[name], type_name='px4_msgs/msg/'+types[name].__name__)
                if len(choices) == 1:
                    if getattr(types[name], 'MESSAGE_VERSION', 0) != (1 if name == 'status' else 0):
                        raise RuntimeError('installed PX4 message contract differs from reviewed v1.16')
                    topics[name] = choices[0]
                    subscriptions[name] = node.create_subscription(types[name], choices[0], lambda msg, key=name: callback(key,msg), qos_profile_sensor_data)
            if settings.get('range_topic') and 'range' not in subscriptions:
                subscriptions['range'] = node.create_subscription(LaserScan, settings['range_topic'], lambda msg: callback('range',msg), qos_profile_sensor_data)
            if plan.task != 'hover' and 'vio_slam' not in subscriptions:
                choices = [topic for topic, found_types in graph
                           if topic == '/vio_shadow/points_slam' and 'sensor_msgs/msg/PointCloud2' in found_types]
                if len(choices) == 1:
                    topics['vio_slam'] = choices[0]
                    subscriptions['vio_slam'] = node.create_subscription(
                        PointCloud2, choices[0], lambda msg: callback('vio_slam', msg), qos_profile_sensor_data)

        def auxiliary_sample(name, sim, wall, max_age=.1):
            stamp, arrival, value = auxiliary.get(name, (None,None,None))
            if stamp is None or not 0 <= sim-stamp <= max_age or wall-arrival > .5:
                raise ValueError(name+' source/arrival not fresh')
            return value

        def actual_odometry(sim, wall):
            value = auxiliary_sample('odom', sim, wall)
            rotation = rotation_wxyz(value['q'])
            if value['pose_frame'] != VehicleOdometry.POSE_FRAME_NED:
                raise ValueError('north-aligned actual PX4 odometry required')
            for key, limit in [('position_variance', .04), ('velocity_variance', .25)]:
                values = np.asarray(value[key])
                if not np.isfinite(values).all() or np.any(values < 0) or np.any(values > limit):
                    raise ValueError('unqualified odometry covariance')
            velocity = np.array(value['velocity'])
            if value['velocity_frame'] == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
                velocity = rotation @ velocity
            elif value['velocity_frame'] != VehicleOdometry.VELOCITY_FRAME_NED:
                raise ValueError('unsupported odometry velocity frame')
            if not np.isfinite([*value['position'], *velocity, *value['angular']]).all():
                raise ValueError('invalid actual odometry')
            if settings.get('max_tilt_rad') is not None and np.arccos(np.clip(rotation[2,2],-1,1))>settings['max_tilt_rad']:
                raise TrackingDeviation('classical attitude exceeds the screened tilt envelope')
            return value, velocity

        def calibrate(sim, wall):
            odom, _ = actual_odometry(sim, wall)
            ranging = auxiliary_sample('range', sim, wall)
            if not ranging['minimum'] <= ranging['distance'] <= ranging['maximum'] or ranging['frame'] != settings['range_frame']:
                raise ValueError('unqualified launch range/frame')
            agl = floor_height_from_range(ranging['distance'], odom['q'], settings['range_offset_frd_m'], settings['range_axis_frd'])
            local = mission.streams['position'].data
            if np.linalg.norm(np.array(odom['position'])-np.array([local[k] for k in ('x','y','z')])) > .03:
                raise ValueError('local-position/odometry origins disagree')
            return CourseFrame.from_launch(position_ned=odom['position'], heading_ned=local['heading'],
                launch_xy=settings['launch_xy_m'], body_agl=agl, physical_heading_in_course=settings['launch_heading_course_rad'],
                placement_reviewed=settings.get('launch_placement_reviewed', False)), agl

        def ensure_publishers():
            if not args.execute or mission.health(now_sim=node.get_clock().now().nanoseconds/1e9, now_wall=time.monotonic()):
                raise RuntimeError('no approved fresh authority for publication')
            if not publishers:
                if 'ack' not in subscriptions:
                    raise RuntimeError('actual native command ACK interface unavailable')
                graph = node.get_topic_names_and_types()
                for name in ('command','heartbeat','trajectory')+ (('rates',) if plan.task == 'ppo' else ()):
                    matches = candidate_topics(graph, direction='in', basename=names[name], type_name='px4_msgs/msg/'+types[name].__name__)
                    if len(matches) != 1 or node.get_publishers_info_by_topic(matches[0]):
                        raise RuntimeError('missing/ambiguous/owned control input '+name)
                    topics[name] = matches[0]
                for name in ('command','heartbeat','trajectory')+ (('rates',) if plan.task == 'ppo' else ()):
                    publishers[name] = node.create_publisher(types[name], topics[name], qos_profile_sensor_data)
            for name in publishers:
                owners=node.get_publishers_info_by_topic(topics[name])
                if len(owners)>1 or any(info.node_name != node.get_name() or info.node_namespace != node.get_namespace()
                       for info in owners):
                    raise RuntimeError('another control owner appeared')

        def stream_position(point, yaw, sim, velocity=None, acceleration=None, yawspeed=None):
            heartbeat = OffboardControlMode(); heartbeat.timestamp=int(sim*1e6); heartbeat.position=True
            message=TrajectorySetpoint(); message.timestamp=heartbeat.timestamp
            message.position=list(point); message.velocity=message.acceleration=message.jerk=[nan]*3
            if velocity is not None:
                velocity=np.asarray(velocity,dtype=float)
                if velocity.shape!=(3,) or not np.isfinite(velocity).all():
                    raise ValueError('finite NED velocity feedforward required')
                message.velocity=velocity.tolist()
            if acceleration is not None:
                acceleration=bounded_feedforward(acceleration, ACCEL_FF_LIMIT)
                message.acceleration=acceleration.tolist()
            message.yaw=float(yaw)
            message.yawspeed=nan if yawspeed is None else float(max(-YAWSPEED_FF_LIMIT,min(YAWSPEED_FF_LIMIT,yawspeed)))
            publishers['trajectory'].publish(message); publishers['heartbeat'].publish(heartbeat)
            mission.streamed(now_sim=sim, now_wall=time.monotonic())

        def navigate(sim, wall):
            nonlocal reference_elapsed, last_nav_sim, route_gate, previous_course, qualification_endpoint_observed
            odom, velocity = actual_odometry(sim, wall)
            course_point = frame.to_course(odom['position'])
            gates = route_gates
            # The original gate function is reused for mission sequencing.
            # This is not an independent physical contact/course grader.
            if previous_course is not None and route_gate < len(gates)-1:
                from drl.obs import gate_update
                passed, missed, _, _ = gate_update(previous_course[None], course_point[None],
                    (frame.ned_to_course@velocity)[None], np.array([route_gate]), gates)
                if missed[0]:
                    raise ValueError('observed route missed a configured gate')
                if passed[0]:
                    gate_events.append({'gate': gates.names[route_gate], 'source_sim_s': auxiliary['odom'][0]})
                    route_gate += 1
            previous_course = course_point
            if plan.task == 'trajectory':
                target, tangent = reference.sample(reference_elapsed)
                target_error=float(np.linalg.norm(course_point-target))
                nearest_phase,nearest_point,error=reference.nearest(course_point,around_s=reference_elapsed)
                tangent_length=float(np.linalg.norm(tangent))
                lag=max(0.,float((target-course_point)@tangent/tangent_length)) if tangent_length>1e-6 else target_error
                _,camera_tangent=reference.sample(min(reference.duration_s,max(reference_elapsed,nearest_phase)+settings.get('heading_lookahead_s',0.)))
                if settings.get('use_reference_yaw'):
                    yaw=frame.north_heading_in_course-reference.yaw(min(reference.duration_s,max(reference_elapsed,nearest_phase)+settings.get('heading_lookahead_s',0.)))
                else:
                    yaw = frame.north_heading_in_course-atan2(camera_tangent[1], camera_tangent[0]) if np.linalg.norm(camera_tangent[:2])>.05 else (tracking_clock.yaw_command if tracking_clock.yaw_command is not None else mission.yaw)
                reference_elapsed,yaw_command=tracking_clock.step(now_sim=sim,error_m=error,desired_yaw=yaw,
                    actual_yaw=mission.streams['position'].data['heading'],progress_floor_s=nearest_phase,lag_error_m=lag)
                reference_elapsed=tracking_clock.elapsed_s=min(reference.duration_s,reference_elapsed)
                target,tangent=reference.sample(reference_elapsed)
                done = (reference_elapsed >= reference.duration_s and np.linalg.norm(course_point-target)<.1 and np.linalg.norm(velocity)<.1)
                if done and bounded_qualification:
                    qualification_endpoint_observed=True
                    log({'kind':'bounded_qualification_endpoint','source_sim_s':sim,
                         'course_point_m':course_point.tolist(),'course_finish_claimed':False})
                    # Use the existing fresh-authority native Land abort path.
                    # This diagnostic must never mark a partial route complete.
                    raise TrackingDeviation('bounded navigation qualification endpoint reached; not a course finish')
                if done and route_gate == len(gates)-1:
                    mission.navigation_finished(evidence='ordered native-estimator gate journal; independent grading pending', outbound_verified=False)
                # A paused virtual clock is a position hold, so its velocity
                # feedforward must be zero. Moving reference uses its actual
                # retimed tangent through ordinary PX4 position control.
                feedforward=frame.ned_to_course.T@tangent*tracking_clock.speed_fraction
                accel_ff=yawspeed_ff=None
                if settings.get('acceleration_feedforward') or settings.get('yawspeed_feedforward'):
                    acc_course,yaw_rate_course=reference_rates(reference,reference_elapsed)
                    sf=tracking_clock.speed_fraction
                    if settings.get('acceleration_feedforward'):
                        # Reference-time derivative scaled by the clock rate squared
                        # (paused clock => zero feedforward, i.e. plain position hold).
                        accel_ff=frame.ned_to_course.T@acc_course*sf*sf
                    if settings.get('yawspeed_feedforward') and settings.get('use_reference_yaw'):
                        # NED yaw = north heading - course yaw.
                        yawspeed_ff=-yaw_rate_course*sf
                stream_position(frame.to_ned(target), yaw_command, sim,velocity=feedforward,
                                acceleration=accel_ff, yawspeed=yawspeed_ff)
                log({'kind':'reference_tracking','source_sim_s':sim,'reference_elapsed_s':reference_elapsed,
                     'error_m':error,'speed_fraction':tracking_clock.speed_fraction,
                     'time_target_error_m':target_error,'nearest_phase_s':nearest_phase,
                     'nearest_point_course_m':nearest_point.tolist(),'lag_error_m':lag,
                     'target_course_m':target.tolist(),'velocity_feedforward_ned_mps':feedforward.tolist()})
            else:
                hover = auxiliary_sample('hover',sim,wall,.5)
                if hover['valid'] is not True:
                    raise ValueError('actual hover-thrust estimate invalid')
                command = policy.step(odom['position'],odom['q'],velocity,odom['angular'],hover['thrust'])
                if command.mode == 'land':
                    if route_gate != len(gates)-1:
                        raise ValueError('policy requested landing before observed gates completed')
                    mission.navigation_finished(evidence='policy pad hover + native-estimator gate journal; independent grading pending', outbound_verified=False)
                    return
                if command.mode != 'fly':
                    raise ValueError('policy core refused: '+command.reason)
                heartbeat=OffboardControlMode(); heartbeat.timestamp=int(sim*1e6); heartbeat.body_rate=True
                message=VehicleRatesSetpoint(); message.timestamp=heartbeat.timestamp
                message.roll,message.pitch,message.yaw=map(float,command.rates_frd); message.thrust_body=[0.,0.,-command.thrust]
                publishers['rates'].publish(message); publishers['heartbeat'].publish(heartbeat)
                mission.streamed(now_sim=sim,now_wall=wall)

        next_tick = next_discovery = started
        try:
            while rclpy.ok():
                wall=time.monotonic()
                if wall >= next_discovery:
                    discover(); next_discovery=wall+.5
                rclpy.spin_once(node, timeout_sec=.01)
                sim=node.get_clock().now().nanoseconds/1e9; wall=time.monotonic()
                if wall-started >= plan.max_wall_s:
                    raise ValueError('bounded mission deadline, including calibration wait')
                if inbox.error:
                    raise ValueError(inbox.error)
                for name, stamp, arrival, value in inbox.drain(sim):
                    if name in mission.streams:
                        mission.observe(name,stamp_s=stamp,received_wall=arrival,data=value)
                    else:
                        auxiliary[name]=(stamp,arrival,value)
                if sim <= 0:
                    if wall - started > vio_quality.startup_timeout_wall_s:
                        raise ValueError('ROS simulation clock not received within bounded startup window')
                    continue  # No /clock yet: nothing may be judged fresh or published.
                quality = vio_quality.status(now_sim_s=sim, now_wall_s=wall)
                if args.execute and plan.task != 'hover' and mission.state == 'WAITING' and quality['state'] != 'READY':
                    if quality['state'] == 'FAULT':
                        raise ValueError('VIO feature preflight failed: '+str(quality['reason']))
                    if wall - started > vio_quality.startup_timeout_wall_s:
                        raise ValueError('OpenVINS SLAM feature preflight did not become ready within its bounded startup window')
                    mission.ready_since = None
                    continue  # Never start the idle-health window or publish until tracks stay adequate.
                if (args.execute and plan.task != 'hover'
                        and mission.state in ('PRESTREAM','WAIT_MODE','DISARMED_CHECK','WAIT_ARM')
                        and quality['state'] != 'READY'):
                    raise ValueError('VIO feature readiness lost before arm: '+str(quality['reason']))
                if args.execute and plan.task != 'hover' and mission.state == 'WAITING' and not navigation_loaded:
                    if mission.health(now_sim=sim,now_wall=wall) is None:
                        try:
                            initial_frame, _ = calibrate(sim,wall)
                        except ValueError:
                            continue  # No publishers exist while calibration is unavailable.
                        sys.path.insert(0,str(args.repo.resolve()))
                        from drl.config import load_config
                        _, _, route_gates = load_config(str(args.repo.resolve()/'config/course_v3.yaml'))
                        if plan.task == 'ppo':
                            policy=TeammatePolicy(args.repo,args.policy,initial_frame)
                        navigation_loaded=True
                if wall >= next_tick:
                    if (mission.state == 'LANDING' and plan.task != 'hover' and mission.abort_reason is None
                            and mission._status().get('disarmed') is True
                            and mission.streams['land'].data.get('landed') is True):
                        odom, _ = actual_odometry(sim,wall)
                        landed_xy=frame.to_course(odom['position'])[:2]
                        if np.linalg.norm(landed_xy-np.array(settings['pad_center_xy_m']))+contract['collision_radius_m'] > settings['pad_outer_radius_m']:
                            raise ValueError('landed vehicle footprint is not inside the pad')
                    actions=mission.step(now_sim=sim,now_wall=wall); next_tick=wall+.02
                    if args.execute and plan.task != 'hover' and frame is None and actions:
                        frame,agl=calibrate(sim,wall)
                        if policy is not None:
                            policy.align(frame)
                        mission.plan=replace(plan,takeoff_m=float(settings['takeoff_agl_m']-agl))
                        log({'kind':'frozen_launch_alignment','north_heading_course':frame.north_heading_in_course,
                             'origin_course':frame.origin_in_course.tolist(),'measured_body_agl_m':agl,'derived_takeoff_m':mission.plan.takeoff_m})
                    if actions:
                        ensure_publishers()
                    for action in actions:
                        quality = vio_quality.status(now_sim_s=node.get_clock().now().nanoseconds/1e9,
                                                     now_wall_s=time.monotonic())
                        if args.execute and plan.task != 'hover' and quality['state'] == 'FAULT':
                            reason = 'VIO feature quality lost: '+str(quality['reason'])
                            if mission.state in ('TAKEOFF', 'NAVIGATE') and mission.owns:
                                abort = mission.abort_for_vio_quality(reason=reason,
                                    now_sim=node.get_clock().now().nanoseconds/1e9, now_wall=time.monotonic())
                                if not abort:
                                    raise RuntimeError('VIO quality loss could not obtain a fresh native Land action')
                                actions.extend(abort)
                                log({'kind': 'vio_quality_abort_requested', 'reason': reason,
                                     'sim_s': node.get_clock().now().nanoseconds/1e9})
                                continue  # Skip this motion action; process the newly appended Land action.
                            if mission.state not in ('LANDING', 'LANDED', 'COMPLETE', 'ABORT_LANDED'):
                                raise ValueError(reason+' before owned flight; publication stopped')
                        if mission.health(now_sim=node.get_clock().now().nanoseconds/1e9,now_wall=time.monotonic()):
                            raise ValueError('fresh authority lost before publication')
                        if action.kind == 'stream':
                            stream_position(action.position,action.yaw,sim,velocity=action.velocity)
                        elif action.kind == 'navigate':
                            try:
                                navigate(sim,wall)
                            except TrackingDeviation as error:
                                # Only a tracking-limit failure with still-fresh
                                # authority may request native Land. Sensor and
                                # authority failures retain the stop-only path.
                                abort=mission.abort_navigation(reason=str(error),now_sim=sim,now_wall=time.monotonic())
                                if abort:
                                    actions.extend(abort)
                        else:
                            message=VehicleCommand(); message.timestamp=int(sim*1e6)
                            message.target_system,message.target_component=mission.target_ids
                            message.source_system,message.source_component=mission.target_ids[0],source_component
                            message.from_external=True
                            if action.kind == 'mode_offboard':
                                message.command=message.VEHICLE_CMD_DO_SET_MODE; message.param1,message.param2=1.,6.
                            elif action.kind == 'arm':
                                message.command=message.VEHICLE_CMD_COMPONENT_ARM_DISARM; message.param1=1.
                            elif action.kind == 'land':
                                message.command=message.VEHICLE_CMD_NAV_LAND
                                message.param4=message.param5=message.param6=message.param7=nan
                            else:
                                raise ValueError('unknown mission command')
                            publishers['command'].publish(message); sent[message.command]=action.kind
                        log({'kind':'action','state':mission.state,'action':action.kind,'sim_s':sim,'wall_s':wall})
                if (mission.state in mission.terminal or mission.state=='LANDED'
                        or not args.execute and wall-started>=args.observe_seconds):
                    break
        except KeyboardInterrupt:
            mission._fault('operator interrupted; publication stopped, landing not established')
        except Exception as error:
            exception=f'{type(error).__name__}: {error}'; mission._fault(exception)
        finally:
            result=mission.report()
            result.update(exception=exception,control_publishers_created=bool(publishers),topics=topics,
                settings_sha256=sha256(args.settings.read_bytes()).hexdigest(),
                airframe_contract_sha256=sha256(args.airframe_contract.read_bytes()).hexdigest(),
                source_sha256=sha256(Path(__file__).read_bytes()).hexdigest(),
                gate_events=gate_events,clock_contract='shared ROS/PX4/Gazebo simulation clock',
                return_execution_qualified=False,independent_course_grade=False,
                vio_feature_quality=vio_quality.report())
            result['bounded_qualification_requested']=bounded_qualification
            result['qualification_endpoint_observed']=qualification_endpoint_observed
            result['commands_published']=list(sent.values())
            result['estimated_airframe']=contract.get('estimated_parameters',False)
            json.dump(json_safe(result),output,indent=2,allow_nan=False)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return 0 if (mission.state=='COMPLETE' if args.execute else mission.state!='FAULT') else 1


if __name__ == '__main__':
    raise SystemExit(main())

