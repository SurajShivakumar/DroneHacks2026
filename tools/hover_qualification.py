"""Reviewable bounded PX4 hover qualification. DEFAULT IS READ-ONLY.

DO NOT RUN --execute until the team approves this exact plan and simulation
reset/ownership/estimator/clearance qualification. Requires --approval-reference
and --shared-simulation-clock; these flags record approval, never grant it.

Plan: 30 s continuous preflight inspection; >1 s Offboard prestream; acknowledged
mode/arm; rise 0.5 m relative to actual PX4 start Z at 0.25 m/s, settle/hold 2 sim
seconds, one NAV_LAND then stop setpoints. Observe Land ACK/mode, landed+disarmed.
Bounds: XY drift0.2 m, relative altitude[-0.1,0.65] m, 30 sim/120 wall seconds.
Freshness budgets reflect measured native cadence: position0.1sim/0.5wall s,
status1sim/2wall s, land2sim/3wall s; source progression must remain advancing.
Heartbeat loop gaps over0.25 s in either clock stop the qualification.
No simulator truth, reset, parameter writes, GCS heartbeat or force arm/disarm.
This is not obstacle avoidance or hardware qualification.

PX4 v1.16: https://docs.px4.io/v1.16/en/ros2/offboard_control
"""
import argparse
from hashlib import sha256
import json
from math import nan
from pathlib import Path
import re
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dronebench.flight_guard import HoverGuard
from dronebench.disarmed_offboard import native_commander_ack_matches


def candidate_topics(graph, *, direction, basename, type_name):
    """Match the reviewed DDS interface and discovered version suffix exactly.

    VehicleCommand also has a mode-executor input; never select it by type alone.
    Ambiguous matching publishers/versions remain a blocking condition.
    """
    pattern = re.compile(r'/fmu/' + re.escape(direction) + '/' + re.escape(basename) + r'(?:_v[0-9]+)?')
    return [topic for topic, types in graph if pattern.fullmatch(topic) and type_name in types]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--approval-reference')
    parser.add_argument('--shared-simulation-clock', action='store_true')
    parser.add_argument('--observe-seconds', type=float, default=35.)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.execute and (not args.approval_reference or not args.approval_reference.strip() or not args.shared_simulation_clock):
        parser.error('execute requires recorded approval-reference and reviewed shared-simulation-clock contract')
    if not 30 <= args.observe_seconds <= 120:
        parser.error('read-only observation must be 30..120 wall seconds')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Evidence files are exclusive and opened before ROS/publishers.
    with args.output.open('x', encoding='utf-8') as report_file, args.output.with_suffix('.jsonl').open('x', encoding='utf-8') as journal:
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, VehicleLandDetected,
                                  VehicleCommandAck, VehicleCommand, OffboardControlMode, TrajectorySetpoint)

        classes = {'status': VehicleStatus, 'position': VehicleLocalPosition, 'land': VehicleLandDetected,
                   'ack': VehicleCommandAck, 'command': VehicleCommand,
                   'heartbeat': OffboardControlMode, 'trajectory': TrajectorySetpoint}
        basenames = {'status': 'vehicle_status', 'position': 'vehicle_local_position', 'land': 'vehicle_land_detected',
                     'ack': 'vehicle_command_ack', 'command': 'vehicle_command',
                     'heartbeat': 'offboard_control_mode', 'trajectory': 'trajectory_setpoint'}
        for name, cls in classes.items():
            expected = 1 if name == 'status' else 0
            if getattr(cls, 'MESSAGE_VERSION', 0) != expected:
                raise RuntimeError(f'{name}: installed message version differs from reviewed v1.16 contract')
        rclpy.init()
        node = rclpy.create_node('dronehacks_hover_qualification')
        node.set_parameters([Parameter('use_sim_time', value=True)])
        started = time.monotonic()
        guard = HoverGuard(started_wall=started, execute=args.execute, approval_reference=args.approval_reference)
        subscriptions, topics, publishers = {}, {}, {}
        sent_commands = {}
        source_component = 191  # MAV_COMP_ID_ONBOARD_COMPUTER; not a fake GCS.
        exception = None

        def log(value):
            journal.write(json.dumps(value, allow_nan=False) + '\n')
            journal.flush()

        def callback(name, message):
            wall = time.monotonic()
            if name == 'status':
                data = {'armed': message.arming_state == message.ARMING_STATE_ARMED,
                        'disarmed': message.arming_state == message.ARMING_STATE_DISARMED,
                        'offboard': message.nav_state == message.NAVIGATION_STATE_OFFBOARD,
                        'land_mode': message.nav_state == message.NAVIGATION_STATE_AUTO_LAND,
                        'startup_mode_allowed': message.nav_state in (message.NAVIGATION_STATE_MANUAL,
                             message.NAVIGATION_STATE_ALTCTL, message.NAVIGATION_STATE_POSCTL, message.NAVIGATION_STATE_AUTO_LOITER),
                        'preflight': message.pre_flight_checks_pass, 'failsafe': message.failsafe,
                        'system_id': message.system_id, 'component_id': message.component_id,
                        'nav_state': message.nav_state, 'arming_state': message.arming_state,
                        'gcs_connection_lost': message.gcs_connection_lost}
                stamp = message.timestamp / 1e6
            elif name == 'position':
                data = {key: getattr(message, key) for key in ('x', 'y', 'z', 'vx', 'vy', 'vz', 'heading',
                        'xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid', 'heading_good_for_control',
                        'xy_reset_counter', 'z_reset_counter', 'vxy_reset_counter', 'vz_reset_counter', 'heading_reset_counter')}
                stamp = message.timestamp_sample / 1e6
            elif name == 'land':
                data = {'landed': message.landed, 'freefall': message.freefall,
                        'ground_contact': message.ground_contact, 'maybe_landed': message.maybe_landed}
                stamp = message.timestamp / 1e6
            else:
                log({'type': 'received_command_ack', 'command': message.command, 'result': message.result,
                     'target_system': message.target_system, 'target_component': message.target_component,
                     'from_external': message.from_external, 'source_s': message.timestamp/1e6, 'wall_s': wall})
                status = guard.streams['status'].data or {}
                kind = sent_commands.get(message.command)
                if not native_commander_ack_matches(message, expected_commands=sent_commands,
                        target_system=status.get('system_id'), target_component=source_component,
                        request_sent=kind is not None):
                    return
                if kind:
                    result = ('accepted' if message.result == message.VEHICLE_CMD_RESULT_ACCEPTED else
                              'in_progress' if message.result == message.VEHICLE_CMD_RESULT_IN_PROGRESS else 'rejected')
                    guard.acknowledge(kind, result=result, stamp_s=message.timestamp / 1e6, received_wall=wall)
                    log({'type': 'ack', 'kind': kind, 'result': result, 'source_s': message.timestamp / 1e6, 'wall_s': wall})
                return
            guard.observe(name, stamp_s=stamp, received_wall=wall, data=data)
            from dronebench.telemetry import json_safe
            log({'type': 'telemetry', 'stream': name, 'source_s': stamp, 'wall_s': wall, 'fields': json_safe(data)})

        def discover():
            graph = node.get_topic_names_and_types()
            for name in ('status', 'position', 'land', 'ack'):
                if name in subscriptions:
                    continue
                typename = 'px4_msgs/msg/' + classes[name].__name__
                choices = candidate_topics(graph, direction='out', basename=basenames[name], type_name=typename)
                if len(choices) == 1:
                    topics[name] = choices[0]
                    subscriptions[name] = node.create_subscription(classes[name], choices[0],
                                                   lambda msg, key=name: callback(key, msg), qos_profile_sensor_data)

        def ensure_publishers():
            if not args.execute:
                raise RuntimeError('read-only mode cannot create control publishers')
            if 'ack' not in subscriptions:
                raise RuntimeError('no unambiguous actual PX4 command ACK output discovered')
            if not publishers:
                graph = node.get_topic_names_and_types()
                for name in ('command', 'heartbeat', 'trajectory'):
                    typename = 'px4_msgs/msg/' + classes[name].__name__
                    choices = candidate_topics(graph, direction='in', basename=basenames[name], type_name=typename)
                    if len(choices) != 1:
                        raise RuntimeError(f'{name}: cannot discover one actual PX4 input topic')
                    topics[name] = choices[0]
                    if node.get_publishers_info_by_topic(choices[0]):
                        raise RuntimeError('another control publisher exists; do not take over')
                # All three inputs are resolved and uncontested before creating any publisher.
                for name in ('command', 'heartbeat', 'trajectory'):
                    publishers[name] = node.create_publisher(classes[name], topics[name], qos_profile_sensor_data)
            for name in ('command', 'heartbeat', 'trajectory'):
                infos = node.get_publishers_info_by_topic(topics[name])
                if len(infos) > 1:
                    raise RuntimeError('multiple command publishers exist; stop publishing')
                for info in infos:
                    if info.node_name != node.get_name() or info.node_namespace != node.get_namespace():
                        raise RuntimeError('another command owner appeared; stop publishing')

        def dispatch(actions, now_sim, now_wall):
            if not actions:
                return
            def recheck():
                reason = guard.health(now_sim=node.get_clock().now().nanoseconds/1e9, now_wall=time.monotonic())
                if reason is not None:
                    raise RuntimeError('telemetry changed before publication: ' + reason)
            recheck()  # Before creating any control publisher, not only at tick time.
            ensure_publishers()
            status = guard.streams['status'].data
            if status['component_id'] == source_component:
                raise RuntimeError('command source component collides with PX4 component')
            for action in actions:
                recheck()
                if action.kind == 'stream':
                    point = TrajectorySetpoint()
                    point.timestamp = int(now_sim * 1e6)
                    point.position = list(action.position)
                    point.velocity = point.acceleration = point.jerk = [nan, nan, nan]
                    point.yaw, point.yawspeed = float(action.yaw), nan
                    heartbeat = OffboardControlMode()
                    heartbeat.timestamp, heartbeat.position = point.timestamp, True
                    publishers['trajectory'].publish(point)
                    publishers['heartbeat'].publish(heartbeat)
                    guard.streamed(now_sim=now_sim, now_wall=now_wall)
                    log({'type': 'setpoint', 'sim_s': now_sim, 'wall_s': now_wall,
                         'position_ned': list(action.position), 'yaw': action.yaw})
                else:
                    command = VehicleCommand()
                    command.timestamp = int(now_sim * 1e6)
                    command.target_system, command.target_component = status['system_id'], status['component_id']
                    command.source_system, command.source_component = status['system_id'], source_component
                    command.from_external, command.confirmation = True, 0
                    if action.kind == 'mode_offboard':
                        command.command = command.VEHICLE_CMD_DO_SET_MODE
                        command.param1, command.param2 = 1., 6.  # Pinned PX4_CUSTOM_MAIN_MODE_OFFBOARD.
                    elif action.kind == 'arm':
                        command.command = command.VEHICLE_CMD_COMPONENT_ARM_DISARM
                        command.param1, command.param2 = 1., 0.  # NO force magic value.
                    elif action.kind == 'land':
                        command.command = command.VEHICLE_CMD_NAV_LAND  # Commander lands at current position.
                    else:
                        raise RuntimeError('unknown command recommendation')
                    sent_commands[command.command] = action.kind
                    publishers['command'].publish(command)
                    log({'type': 'command', 'kind': action.kind, 'command': command.command,
                         'sim_s': now_sim, 'wall_s': now_wall})

        next_discovery = next_tick = started
        try:
            while rclpy.ok():
                wall = time.monotonic()
                if wall >= next_discovery:
                    discover()
                    next_discovery = wall + .5
                rclpy.spin_once(node, timeout_sec=.02)
                wall = time.monotonic()
                now_sim = node.get_clock().now().nanoseconds / 1e9
                if wall >= next_tick:
                    actions = guard.step(now_sim=now_sim, now_wall=wall)
                    dispatch(actions, now_sim, wall)
                    next_tick = wall + .05
                if guard.state in guard.terminal or (not args.execute and wall - started >= args.observe_seconds):
                    break
        except KeyboardInterrupt:
            try:
                now_sim, now_wall = node.get_clock().now().nanoseconds/1e9, time.monotonic()
                dispatch(guard.abort('operator interrupt', now_sim=now_sim, now_wall=now_wall), now_sim, now_wall)
            except Exception as exc:
                exception = f'interrupt handoff failed: {type(exc).__name__}: {exc}'
                guard.state, guard.reason, guard.owns = 'FAULT', exception, False
        except Exception as exc:
            exception = f'{type(exc).__name__}: {exc}'
            guard.state, guard.reason, guard.owns = 'FAULT', exception, False
        finally:
            from dronebench.telemetry import json_safe
            result = guard.report(now_sim=node.get_clock().now().nanoseconds/1e9, now_wall=time.monotonic())
            result.update(kind='bounded_hover_qualification' if args.execute else 'read_only_preflight_observation',
                          exception=exception, topics=topics, control_publishers_created=bool(publishers),
                          wall_elapsed_s=time.monotonic()-started,
                          code_sha256={'hover_qualification.py': sha256(Path(__file__).read_bytes()).hexdigest(),
                                       'flight_guard.py': sha256((Path(__file__).resolve().parents[1]/'dronebench/flight_guard.py').read_bytes()).hexdigest()},
                          flight_ready_claimed=False, successful_qualification=guard.state == 'COMPLETE')
            json.dump(json_safe(result), report_file, indent=2, allow_nan=False)
            report_file.write('\n')
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return 0 if (guard.state == 'COMPLETE' if args.execute else exception is None) else 1


if __name__ == '__main__':
    raise SystemExit(main())
