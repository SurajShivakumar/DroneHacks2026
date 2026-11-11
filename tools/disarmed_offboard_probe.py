"""Read-only by default; optional DISARMED Offboard-mode diagnostic, never flight.

--probe-mode --shared-simulation-clock permits only: a constant measured current
NED position/yaw hold heartbeat, followed by one Offboard mode request. It first
observes 30 continuous wall seconds of fresh landed/disarmed AUTO_LOITER health.
After a matching command ACK and actual Offboard status, it observes the native
PX4 pre_flight_checks_pass flag for 30 continuous wall seconds while disarmed.
No arm, Land, desired movement, parameter writes, fake fixes or GCS heartbeats.
All output stops on stale/armed/authority/reset/identity/movement failures. Total
wall budget120 seconds. A completed diagnostic does not approve or qualify flight.
Stopping the heartbeat can change the disarmed PX4 mode after this tool exits.
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
from dronebench.disarmed_offboard import DisarmedOffboardProbe, native_commander_ack_matches
from dronebench.flight_guard import HoverGuard
from dronebench.telemetry import json_safe
from dronebench.causal_telemetry import CausalInbox


def candidate_topics(graph, *, direction, basename, type_name):
    pattern = re.compile('/fmu/' + re.escape(direction) + '/' + re.escape(basename) + r'(?:_v[0-9]+)?')
    return [topic for topic, types in graph if pattern.fullmatch(topic) and type_name in types]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe-mode', action='store_true')
    parser.add_argument('--shared-simulation-clock', action='store_true')
    parser.add_argument('--observe-seconds', type=float, default=35.)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.probe_mode and not args.shared_simulation_clock:
        parser.error('probe-mode requires the reviewed shared-simulation-clock contract')
    if not 30 <= args.observe_seconds <= 120:
        parser.error('read-only observation must be 30..120 wall seconds')
    args.output.parent.mkdir(parents=True, exist_ok=True)
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
            if getattr(cls, 'MESSAGE_VERSION', 0) != (1 if name == 'status' else 0):
                raise RuntimeError(f'{name}: installed message version differs from reviewed PX4 v1.16')
        rclpy.init()
        node = rclpy.create_node('dronehacks_disarmed_offboard_probe')
        node.set_parameters([Parameter('use_sim_time', value=True)])
        started = time.monotonic()
        probe = DisarmedOffboardProbe(started_wall=started, probe_mode=args.probe_mode)
        inbox = CausalInbox()
        subscriptions, topics, publishers = {}, {}, {}
        mode_sent = False
        source_component = 191
        exception = None

        def log(value):
            journal.write(json.dumps(json_safe(value), allow_nan=False) + '\n')
            journal.flush()

        def callback(name, msg):
            wall = time.monotonic()
            if name == 'status':
                fields = {'armed': msg.arming_state == msg.ARMING_STATE_ARMED,
                          'disarmed': msg.arming_state == msg.ARMING_STATE_DISARMED,
                          'offboard': msg.nav_state == msg.NAVIGATION_STATE_OFFBOARD,
                          'auto_loiter': msg.nav_state == msg.NAVIGATION_STATE_AUTO_LOITER,
                          'preflight': msg.pre_flight_checks_pass, 'failsafe': msg.failsafe,
                          'system_id': msg.system_id, 'component_id': msg.component_id,
                          'nav_state': msg.nav_state, 'arming_state': msg.arming_state,
                          'gcs_connection_lost': msg.gcs_connection_lost}
                stamp = msg.timestamp/1e6
            elif name == 'position':
                keys = ('x', 'y', 'z', 'vx', 'vy', 'vz', 'heading', 'xy_valid', 'z_valid', 'v_xy_valid',
                        'v_z_valid', 'heading_good_for_control') + HoverGuard.reset_fields
                fields = {key: getattr(msg, key) for key in keys}
                stamp = msg.timestamp_sample/1e6
            elif name == 'land':
                fields = {'landed': msg.landed, 'freefall': msg.freefall, 'ground_contact': msg.ground_contact}
                stamp = msg.timestamp/1e6
            else:
                log({'type': 'received_command_ack', 'command': msg.command, 'result': msg.result,
                     'target_system': msg.target_system, 'target_component': msg.target_component,
                     'from_external': msg.from_external, 'source_s': msg.timestamp/1e6,
                     'wall_s': wall, 'mode_request_sent': mode_sent})
                status = probe.status()
                if not native_commander_ack_matches(msg,
                        expected_commands=(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,),
                        target_system=status.get('system_id'), target_component=source_component, request_sent=mode_sent):
                    return
                result = ('accepted' if msg.result == msg.VEHICLE_CMD_RESULT_ACCEPTED else
                          'in_progress' if msg.result == msg.VEHICLE_CMD_RESULT_IN_PROGRESS else 'rejected')
                probe.acknowledge(result=result, stamp_s=msg.timestamp/1e6, received_wall=wall)
                log({'type': 'mode_ack', 'result': result, 'source_s': msg.timestamp/1e6, 'wall_s': wall})
                return
            inbox.put(name, stamp, wall, fields)
            log({'type': 'telemetry', 'stream': name, 'source_s': stamp, 'wall_s': wall, 'fields': fields})

        def discover():
            graph = node.get_topic_names_and_types()
            for name in ('status', 'position', 'land', 'ack'):
                if name in subscriptions:
                    continue
                choices = candidate_topics(graph, direction='out', basename=basenames[name],
                                             type_name='px4_msgs/msg/' + classes[name].__name__)
                if len(choices) == 1:
                    topics[name] = choices[0]
                    subscriptions[name] = node.create_subscription(classes[name], choices[0],
                                               lambda msg, key=name: callback(key, msg), qos_profile_sensor_data)

        def current_time():
            return node.get_clock().now().nanoseconds/1e9, time.monotonic()

        def recheck():
            sim, wall = current_time()
            error = probe.publication_error(now_sim=sim, now_wall=wall)
            if error:
                raise RuntimeError('publication stopped: ' + error)
            if probe.status()['component_id'] == source_component:
                raise RuntimeError('command source component collides with PX4')

        def ensure_publishers():
            if not args.probe_mode:
                raise RuntimeError('read-only diagnostic cannot create control publishers')
            recheck()
            if 'ack' not in subscriptions:
                raise RuntimeError('no unambiguous actual PX4 mode-command ACK topic discovered')
            if not publishers:
                graph = node.get_topic_names_and_types()
                for name in ('command', 'heartbeat', 'trajectory'):
                    choices = candidate_topics(graph, direction='in', basename=basenames[name],
                                                 type_name='px4_msgs/msg/' + classes[name].__name__)
                    if len(choices) != 1:
                        raise RuntimeError(f'{name}: input topic missing or ambiguous')
                    topics[name] = choices[0]
                    if node.get_publishers_info_by_topic(choices[0]):
                        raise RuntimeError('another control publisher exists; diagnostic cannot take over')
                recheck()
                for name in ('command', 'heartbeat', 'trajectory'):
                    publishers[name] = node.create_publisher(classes[name], topics[name], qos_profile_sensor_data)
            for name in ('command', 'heartbeat', 'trajectory'):
                infos = node.get_publishers_info_by_topic(topics[name])
                if len(infos) > 1 or any(info.node_name != node.get_name() or info.node_namespace != node.get_namespace() for info in infos):
                    raise RuntimeError('another command owner appeared; stop all output')

        def dispatch(actions):
            nonlocal mode_sent
            if not actions:
                return
            ensure_publishers()
            for action in actions:
                recheck()
                sim, wall = current_time()
                if action.kind == 'stream':
                    point = TrajectorySetpoint()
                    point.timestamp = int(sim*1e6)
                    point.position = list(action.position)
                    point.velocity = point.acceleration = point.jerk = [nan, nan, nan]
                    point.yaw, point.yawspeed = float(action.yaw), nan
                    heartbeat = OffboardControlMode()
                    heartbeat.timestamp, heartbeat.position = point.timestamp, True
                    publishers['trajectory'].publish(point)
                    publishers['heartbeat'].publish(heartbeat)
                    probe.streamed(now_sim=sim, now_wall=wall)
                    log({'type': 'constant_position_hold', 'sim_s': sim, 'wall_s': wall,
                         'position_ned': action.position, 'yaw': action.yaw})
                elif action.kind == 'mode_offboard':
                    if mode_sent:
                        raise RuntimeError('duplicate mode request prohibited')
                    status = probe.status()
                    command = VehicleCommand()
                    command.timestamp = int(sim*1e6)
                    command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
                    command.param1, command.param2 = 1., 6.  # Pinned PX4 custom mode Offboard.
                    command.target_system, command.target_component = status['system_id'], status['component_id']
                    command.source_system, command.source_component = status['system_id'], source_component
                    command.from_external, command.confirmation = True, 0
                    publishers['command'].publish(command)
                    mode_sent = True
                    log({'type': 'offboard_mode_only', 'sim_s': sim, 'wall_s': wall, 'command': command.command})
                else:
                    raise RuntimeError('action outside disarmed diagnostic scope')

        next_discovery = next_tick = started
        logged_events = 0
        try:
            while rclpy.ok():
                wall = time.monotonic()
                if wall >= next_discovery:
                    discover()
                    next_discovery = wall+.5
                rclpy.spin_once(node, timeout_sec=.02)
                sim, wall = current_time()
                if inbox.error:
                    raise RuntimeError(inbox.error)
                for name, stamp, arrival, fields in inbox.drain(sim):
                    probe.observe(name, stamp_s=stamp, received_wall=arrival, data=fields)
                if wall >= next_tick:
                    actions = probe.step(now_sim=sim, now_wall=wall)
                    for event in probe.events[logged_events:]:
                        log({'type': 'guard_checkpoint', **event})
                    logged_events = len(probe.events)
                    dispatch(actions)
                    next_tick = wall+.05
                if probe.state in probe.terminal or not args.probe_mode and wall-started >= args.observe_seconds:
                    break
        except KeyboardInterrupt:
            probe.state, probe.reason = 'FAULT', 'operator interrupted disarmed diagnostic'
        except Exception as exc:
            exception = f'{type(exc).__name__}: {exc}'
            probe.state, probe.reason = 'FAULT', exception
        finally:
            result = probe.report()
            result.update(exception=exception, topics=topics, control_publishers_created=bool(publishers),
                          mode_commands_sent=int(mode_sent), wall_elapsed_s=time.monotonic()-started,
                          code_sha256={path.name: sha256(path.read_bytes()).hexdigest() for path in
                              (Path(__file__), Path(__file__).resolve().parents[1]/'dronebench/disarmed_offboard.py',
                               Path(__file__).resolve().parents[1]/'dronebench/flight_guard.py')})
            json.dump(json_safe(result), report_file, indent=2, allow_nan=False)
            report_file.write('\n')
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return 0 if (probe.state == 'COMPLETE' if args.probe_mode else probe.state != 'FAULT') else 1


if __name__ == '__main__':
    raise SystemExit(main())
