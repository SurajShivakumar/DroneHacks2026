"""Bounded real OpenVINS odometry into PX4; disarmed validation by default mode.

No arming, offboard or trajectory publisher exists. Shared ROS/Gazebo simulation
clock and coincident proxy IMU/body origins are explicit test contracts. This
Explicit approved flight-support mode continues sensor estimates during arming,
Land and authority changes; it never publishes a control/arm/mode command.
This adapter is not qualified for hardware clocks.
"""
import argparse
from collections import Counter, deque
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dronebench.vision_odometry import OdometrySample, VisionConverter
from dronebench.heading_alignment import HeadingAlignment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=110)
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--disarmed-validation', action='store_true')
    mode.add_argument('--flight-support', action='store_true')
    parser.add_argument('--approval-reference')
    parser.add_argument('--shared-simulation-clock',action='store_true')
    parser.add_argument('--align-heading-from-px4', action='store_true')
    parser.add_argument('--airframe-contract',type=Path,
                        help='Explicit calibrated/estimated body_from_imu transform; never infer zero offset')
    args = parser.parse_args()
    if args.flight_support and not (args.approval_reference and args.approval_reference.strip() and args.shared_simulation_clock and args.airframe_contract and args.align_heading_from_px4):
        parser.error('flight-support requires recorded approval, shared clock, airframe transform and heading alignment')
    body_from_imu=None
    if args.airframe_contract:
        body_from_imu=json.loads(args.airframe_contract.read_text())['body_from_imu']
    if not 30 <= args.seconds <= 1210:
        parser.error('bounded sensor support must last30..1210 wall seconds')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as report:
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from px4_msgs.msg import VehicleOdometry, VehicleStatus, VehicleAttitude, EstimatorStatusFlags
        rclpy.init()
        node = rclpy.create_node('dronebench_disarmed_vision_validation')
        node.set_parameters([Parameter('use_sim_time', value=True)])
        pub = node.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', qos_profile_sensor_data)
        queue, drops = deque(maxlen=50), Counter()
        converter = None if args.align_heading_from_px4 else VisionConverter(clock_domain='ros_sim', pose_frame='FRD',body_from_imu=body_from_imu)
        heading = HeadingAlignment()
        attitudes = deque(maxlen=200)
        state = {'arming_state': None, 'status_stamp': None, 'status_progress_wall': None, 'target_ids':None,
                 'first_odom_s': None, 'published': 0, 'received': 0, 'fault': None,
                 'heading_alignment': None, 'estimator_flags': {}, 'flags_stamp_s': 0.}
        subscribers = {}

        def status(msg):
            ids=(msg.system_id,msg.component_id)
            if state['target_ids'] is not None and tuple(state['target_ids'])!=ids:
                state['fault']='PX4 identity changed; sensor bridge stopped'
            state['target_ids']=ids
            state['arming_state'] = msg.arming_state
            if not args.flight_support and msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                state['fault'] = 'armed authority observed; vision validation stopped'
            if state['status_stamp'] is None or msg.timestamp > state['status_stamp']:
                state['status_progress_wall'] = time.monotonic()
                state['status_stamp'] = msg.timestamp

        def odometry(msg):
            state['received'] += 1
            sample = OdometrySample.from_ros(msg, clock_domain='ros_sim')
            queue.append((sample, time.monotonic()))

        def attitude(msg):
            attitudes.append((msg.timestamp/1e6, tuple(float(x) for x in msg.q), time.monotonic()))

        def estimator_flags(msg):
            names = ('cs_yaw_align', 'cs_tilt_align', 'cs_ev_yaw', 'cs_mag_hdg', 'cs_mag_3d',
                     'cs_mag_field_disturbed', 'cs_mag_fault')
            state['estimator_flags'] = {key: getattr(msg, key) for key in names}
            state['flags_stamp_s'] = msg.timestamp / 1e6

        subscribers['odom'] = node.create_subscription(Odometry, '/vio_shadow/odomimu', odometry, qos_profile_sensor_data)

        def publish():
            nonlocal converter
            if state['fault']:
                return
            accepted_states=(VehicleStatus.ARMING_STATE_DISARMED,VehicleStatus.ARMING_STATE_ARMED) if args.flight_support else (VehicleStatus.ARMING_STATE_DISARMED,)
            if (state['arming_state'] not in accepted_states
                    or state['status_progress_wall'] is None
                    or time.monotonic() - state['status_progress_wall'] > (2. if args.flight_support else .5)):
                drops['disarmed_telemetry_not_fresh'] += 1
                return
            now = node.get_clock().now().nanoseconds / 1e9
            if args.flight_support and not 0 <= now-state['status_stamp']/1e6 <= 1.:
                drops['status_source_not_fresh']+=1
                return
            if args.flight_support and state['arming_state']==VehicleStatus.ARMING_STATE_ARMED and converter is None:
                state['fault']='armed before sensor heading alignment was established'
                return
            candidates = [(s, wall) for s, wall in queue if s.stamp_s <= now]
            if not candidates:
                drops['no_causally_available_odom'] += 1
                return
            sample, wall = candidates[-1]
            if time.monotonic() - wall > .5:
                drops['odom_arrival_stale'] += 1
                return
            if state['first_odom_s'] is None:
                state['first_odom_s'] = sample.stamp_s
            if sample.stamp_s - state['first_odom_s'] < 1.:
                drops['initial_observation_window'] += 1
                return
            if converter is None:
                available = [entry for entry in attitudes if entry[0] <= now]
                if not available:
                    drops['no_qualified_PX4_attitude_for_heading'] += 1
                    return
                att_stamp, att_q, att_wall = min(available, key=lambda entry: abs(entry[0]-sample.stamp_s))
                if time.monotonic()-att_wall > .5:
                    drops['heading_attitude_arrival_stale'] += 1
                    return
                try:
                    alignment = heading.observe(vio_stamp_s=sample.stamp_s, vio_xyzw=sample.orientation_xyzw,
                        attitude_stamp_s=att_stamp, attitude_wxyz=att_q, now_s=now,
                        flags=state['estimator_flags'], flags_stamp_s=state['flags_stamp_s'])
                except ValueError as exc:
                    drops[str(exc)] += 1
                    return
                if alignment is None:
                    drops['measuring_initial_heading_alignment'] += 1
                    return
                converter = VisionConverter(clock_domain='ros_sim', pose_frame='NED', world_to_enu=alignment,body_from_imu=body_from_imu)
                state['heading_alignment'] = heading.metadata
            try:
                out = converter.convert(sample, now_s=now, now_clock_domain='ros_sim')
                if max(out.position_variance) > .04:
                    raise ValueError('position uncertainty exceeds validation envelope')
            except ValueError as exc:
                drops[str(exc)] += 1
                return
            msg = VehicleOdometry()
            msg.timestamp, msg.timestamp_sample = out.timestamp_us, out.timestamp_sample_us
            msg.pose_frame = getattr(VehicleOdometry, 'POSE_FRAME_' + out.pose_frame)
            msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
            msg.position, msg.q = list(out.position), list(out.q_wxyz)
            msg.velocity, msg.angular_velocity = list(out.velocity), list(out.angular_velocity)
            msg.position_variance = list(out.position_variance)
            msg.orientation_variance = list(out.orientation_variance)
            msg.velocity_variance = list(out.velocity_variance)
            msg.reset_counter = 0  # One continuous test process; converter rejects time resets.
            msg.quality = 0  # Unknown/unset; never invent a confidence percentage.
            pub.publish(msg)
            state['published'] += 1

        node.create_timer(1/30, publish)
        started, next_discovery = time.monotonic(), 0.
        try:
            while time.monotonic() - started < args.seconds and not state['fault']:
                if time.monotonic() > next_discovery:
                    for topic, types in node.get_topic_names_and_types():
                        if 'px4_msgs/msg/VehicleStatus' in types and topic not in subscribers:
                            subscribers[topic] = node.create_subscription(VehicleStatus, topic, status, qos_profile_sensor_data)
                        if 'px4_msgs/msg/VehicleAttitude' in types and topic not in subscribers:
                            subscribers[topic] = node.create_subscription(VehicleAttitude, topic, attitude, qos_profile_sensor_data)
                        if 'px4_msgs/msg/EstimatorStatusFlags' in types and topic not in subscribers:
                            subscribers[topic] = node.create_subscription(EstimatorStatusFlags, topic, estimator_flags, qos_profile_sensor_data)
                    next_discovery = time.monotonic() + 2
                rclpy.spin_once(node, timeout_sec=.05)
        except KeyboardInterrupt:
            pass
        except rclpy.executors.ExternalShutdownException:
            pass
        finally:
            json.dump({'kind': 'approved_external_vision_sensor_support' if args.flight_support else 'disarmed_external_vision_validation_not_flight',
                       'flight_support':args.flight_support,'approval_reference':args.approval_reference,
                       'body_from_imu':body_from_imu,
                       'duration_wall_s': time.monotonic()-started, **state, 'drops': dict(drops),
                       'pose_frame': 'calibrated NED' if args.align_heading_from_px4 else 'arbitrary-heading FRD', 'quality': 'unknown/unset',
                       'clock_contract': 'shared ROS/Gazebo simulation time',
                       'hardware_qualified': False, 'reset_continuity_qualified': False}, report, indent=2)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    if state['fault'] or not state['published']:
        raise RuntimeError('No valid disarmed vision publication; inspect report')


if __name__ == '__main__':
    main()
