"""SIM ONLY: stands in for the Sky Beta safety pilot (organizer competitor guide §4).

Real procedure: stack up and still; the pilot ARMS with the mode switch on
Altitude, then switches to POSITION on the ground; the organizer flight_interface
(auto_offboard_on_arm) then requests Offboard once and the team node flies,
including takeoff. The pilot takes the vehicle back (here: commands Land) if
Offboard ends before the team node has landed, or on a wall deadline.

Uses PX4 vehicle_command, which on the real drone only the RC pilot does; this
process must never run on hardware. Writes a mission report in the shape our
truth evaluator reads (events: armed_observed, navigation_started,
request_land, landed_disarmed) plus the full /skybeta/state history summary.

--sticks: the pilot's transmitter. PX4 1.16 lets Altitude and Position be armed
only with a valid manual-control source (mode_requirements.cpp: both modes
require mode_req_manual_control; modeCheck.cpp clears their arming bits while
manual_control_signal_lost). The real pilot's RC provides it; in SITL this
process publishes the same stick state the pilot holds (throttle fully down,
sticks centred, never moving) as a MAVLink-source manual_control_input, which
PX4 accepts with COM_RC_IN_MODE=1 (joystick only, the SITL default). It keeps
publishing for the whole run, as the transmitter stays on in the real flight.

Diagnostics: a one-line snapshot of PX4's arming-relevant state every
--snapshot-s seconds and every vehicle_command_ack go to stdout and the report.
"""
import argparse
import json
from pathlib import Path
import sys
import time

NAV_STATE = {0: 'MANUAL', 1: 'ALTCTL', 2: 'POSCTL', 3: 'AUTO_MISSION', 4: 'AUTO_LOITER', 5: 'AUTO_RTL',
             10: 'ACRO', 12: 'DESCEND', 13: 'TERMINATION', 14: 'OFFBOARD', 15: 'STAB', 17: 'AUTO_TAKEOFF',
             18: 'AUTO_LAND', 19: 'AUTO_FOLLOW', 20: 'AUTO_PRECLAND', 21: 'ORBIT', 22: 'AUTO_VTOL_TAKEOFF'}
ACK_RESULT = {0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED', 4: 'FAILED',
              5: 'IN_PROGRESS', 6: 'CANCELLED'}
CUSTOM_MAIN_MODE = {1: 'MANUAL', 2: 'ALTCTL', 3: 'POSCTL', 4: 'AUTO', 5: 'ACRO', 6: 'OFFBOARD', 7: 'STAB'}
# failsafe_flags booleans that block arming or a mode (names from px4_msgs 1.16 FailsafeFlags.msg)
FAILSAFE_BOOLS = ('angular_velocity_invalid', 'attitude_invalid', 'local_altitude_invalid', 'local_position_invalid',
                  'local_position_invalid_relaxed', 'local_velocity_invalid', 'global_position_invalid',
                  'offboard_control_signal_lost', 'home_position_invalid', 'manual_control_signal_lost',
                  'gcs_connection_lost', 'battery_unhealthy', 'geofence_breached', 'fd_critical_failure',
                  'fd_esc_arming_failure', 'local_position_accuracy_low')
EKF_FLAGS = ('cs_tilt_align', 'cs_yaw_align', 'cs_ev_pos', 'cs_ev_yaw', 'cs_ev_vel', 'cs_ev_hgt', 'cs_baro_hgt',
             'cs_rng_hgt', 'cs_mag_hdg', 'cs_mag_3d', 'cs_mag', 'cs_ev_yaw_fault', 'cs_mag_fault',
             'cs_inertial_dead_reckoning', 'cs_vehicle_at_rest', 'cs_fake_pos', 'cs_in_air')


def mode_bits(mask):
    return ','.join(NAV_STATE.get(i, str(i)) for i in range(32) if mask >> i & 1) or '-'


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--ready-hold-s', type=float, default=20., help='odom + team setpoints steady before arming')
    p.add_argument('--max-flight-s', type=float, default=400., help='sim seconds after arming before pilot lands')
    p.add_argument('--max-wall-s', type=float, default=900.)
    p.add_argument('--arm-timeout-s', type=float, default=60., help='wall seconds of refused arming before giving up')
    p.add_argument('--snapshot-s', type=float, default=5., help='sim seconds between diagnostic snapshots')
    p.add_argument('--sticks', action='store_true',
                   help="SIM ONLY: publish the pilot's transmitter state (throttle down, sticks centred) to PX4")
    a = p.parse_args()
    sys.stdout.reconfigure(line_buffering=True)

    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from px4_msgs.msg import (VehicleCommand, VehicleCommandAck, VehicleStatus, FailsafeFlags, EstimatorStatusFlags,
                              VehicleLocalPosition, ManualControlSetpoint)
    from skybeta_flight_interface.topics import resolve_endpoints, fmu_topic

    rclpy.init()
    node = rclpy.create_node('sim_pilot_emulator')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    topics = resolve_endpoints()
    qos_px4 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
    cmd_pub = node.create_publisher(VehicleCommand, topics['vehicle_command'], qos_px4)
    sticks_pub = node.create_publisher(ManualControlSetpoint, fmu_topic('in', 'manual_control_input', 'ManualControlSetpoint'),
                                       qos_px4) if a.sticks else None
    st = {'state': None, 'last_sp_wall': None, 'phase': 'WAIT_READY', 'ready_since': None,
          'armed_at': None, 'events': [], 'offboard_seen': False, 'land_sent': False, 'reason': None,
          'states': 0, 'max_clamped': set(), 'started_wall': time.monotonic(),
          'px4': {}, 'snapshots': [], 'acks': [], 'commands': [], 'last_snapshot': None, 'sticks_sent': 0}

    def sim_now():
        return node.get_clock().now().nanoseconds*1e-9

    def say(text):
        print(f'[pilot sim={sim_now():8.2f}] {text}', flush=True)

    def event(kind, **kw):
        st['events'].append({'kind': kind, 'sim_s': sim_now(), 'wall_s': time.monotonic(), **kw})
        say(f'event {kind} {kw}')

    def command(cmd, p1=0., p2=0., p3=0., note=''):
        m = VehicleCommand()
        m.timestamp = int(sim_now()*1e6)
        m.command, m.param1, m.param2, m.param3 = cmd, float(p1), float(p2), float(p3)
        m.target_system, m.target_component, m.source_system, m.source_component = 1, 1, 255, 1
        m.from_external = True
        cmd_pub.publish(m)
        rec = {'sim_s': sim_now(), 'command': int(cmd), 'param1': float(p1), 'param2': float(p2), 'param3': float(p3), 'note': note}
        if len(st['commands']) < 400:
            st['commands'].append(rec)
        say(f'sent VehicleCommand {cmd} p1={p1:g} p2={p2:g} p3={p3:g} {note}')

    def set_mode(main, sub=0):
        # MAV_CMD_DO_SET_MODE: param1 base mode flags (1 = custom mode enabled), param2 PX4 custom main mode,
        # param3 custom sub mode (AUTO: 6 = LAND).
        command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., float(main), float(sub),
                note=f'set_mode {CUSTOM_MAIN_MODE.get(main, main)}' + (f' sub={sub}' if sub else ''))

    def arm():
        command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1., note='arm')

    def on_state(msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        st['state'] = s
        st['states'] += 1
        for c in s.get('clamped') or []:
            st['max_clamped'].add(c)

    def on_sp(_msg):
        st['last_sp_wall'] = time.monotonic()

    def on_status(m):
        st['px4']['status'] = {'nav_state': int(m.nav_state), 'arming_state': int(m.arming_state),
                               'pre_flight_checks_pass': bool(m.pre_flight_checks_pass), 'failsafe': bool(m.failsafe),
                               'valid_nav_states_mask': int(m.valid_nav_states_mask),
                               'can_set_nav_states_mask': int(m.can_set_nav_states_mask),
                               'gcs_connection_lost': bool(m.gcs_connection_lost)}

    def on_failsafe(m):
        st['px4']['failsafe'] = {'set': [n for n in FAILSAFE_BOOLS if getattr(m, n)],
                                 'mode_req_manual_control': int(m.mode_req_manual_control),
                                 'mode_req_local_position_relaxed': int(m.mode_req_local_position_relaxed),
                                 'mode_req_local_alt': int(m.mode_req_local_alt),
                                 'mode_req_prevent_arming': int(m.mode_req_prevent_arming)}

    def on_ekf(m):
        st['px4']['ekf'] = [n for n in EKF_FLAGS if getattr(m, n)]

    def on_lpos(m):
        st['px4']['lpos'] = {'xy_valid': bool(m.xy_valid), 'z_valid': bool(m.z_valid), 'v_xy_valid': bool(m.v_xy_valid),
                             'v_z_valid': bool(m.v_z_valid), 'heading_good_for_control': bool(m.heading_good_for_control),
                             'dead_reckoning': bool(m.dead_reckoning), 'eph': round(float(m.eph), 3),
                             'epv': round(float(m.epv), 3), 'z': round(float(m.z), 3)}

    def on_ack(m):
        rec = {'sim_s': sim_now(), 'command': int(m.command), 'result': int(m.result),
               'result_name': ACK_RESULT.get(int(m.result), str(int(m.result))),
               'result_param1': int(m.result_param1), 'result_param2': int(m.result_param2),
               'from_external': bool(m.from_external)}
        if len(st['acks']) < 400:
            st['acks'].append(rec)
        say(f"ack command={rec['command']} result={rec['result_name']} p1={rec['result_param1']} p2={rec['result_param2']}")

    node.create_subscription(String, '/skybeta/state', on_state, 10)
    node.create_subscription(PoseStamped, '/skybeta/setpoint/pose', on_sp, 10)
    node.create_subscription(TwistStamped, '/skybeta/setpoint/twist', on_sp, 10)
    node.create_subscription(VehicleStatus, topics['vehicle_status'], on_status, qos_px4)
    node.create_subscription(FailsafeFlags, fmu_topic('out', 'failsafe_flags', 'FailsafeFlags'), on_failsafe, qos_px4)
    node.create_subscription(EstimatorStatusFlags, fmu_topic('out', 'estimator_status_flags', 'EstimatorStatusFlags'), on_ekf, qos_px4)
    node.create_subscription(VehicleLocalPosition, topics['vehicle_local_position'], on_lpos, qos_px4)
    node.create_subscription(VehicleCommandAck, fmu_topic('out', 'vehicle_command_ack', 'VehicleCommandAck'), on_ack, qos_px4)

    def snapshot(force=False):
        t = sim_now()
        if not force and st['last_snapshot'] is not None and t-st['last_snapshot'] < a.snapshot_s:
            return
        st['last_snapshot'] = t
        px, s = st['px4'], st['state'] or {}
        stat, fs, lp = px.get('status') or {}, px.get('failsafe') or {}, px.get('lpos') or {}
        rec = {'sim_s': t, 'phase': st['phase'], 'px4': json.loads(json.dumps(px)),
               'skybeta': {k: s.get(k) for k in ('px4_link', 'armed', 'landed', 'nav_state_name', 'offboard_active')},
               'fresh_setpoint': st['last_sp_wall'] is not None and time.monotonic()-st['last_sp_wall'] < 1.}
        if len(st['snapshots']) < 600:
            st['snapshots'].append(rec)
        say(f"snapshot phase={st['phase']} nav={NAV_STATE.get(stat.get('nav_state'), stat.get('nav_state'))} "
            f"arming_state={stat.get('arming_state')} preflight_pass={stat.get('pre_flight_checks_pass')} "
            f"can_arm_modes=[{mode_bits(stat.get('valid_nav_states_mask', 0))}] "
            f"failsafe_set={fs.get('set')} manual_req_modes=[{mode_bits(fs.get('mode_req_manual_control', 0))}] "
            f"lpos={lp} ekf={px.get('ekf')} skybeta={rec['skybeta']} fresh_sp={rec['fresh_setpoint']} "
            f"sticks_sent={st['sticks_sent']}")

    def finish(state, reason=None):
        snapshot(force=True)
        s = st['state'] or {}
        report = {'kind': 'skybeta_profile_sim_run', 'state': state, 'reason': reason, 'events': st['events'],
                  'offboard_seen': st['offboard_seen'], 'clamp_reasons_seen': sorted(st['max_clamped']),
                  'final_state': s, 'state_messages': st['states'],
                  'sim_sticks_published': bool(sticks_pub), 'sticks_messages': st['sticks_sent'],
                  'diagnostics': {'commands': st['commands'], 'acks': st['acks'], 'snapshots': st['snapshots']},
                  'note': 'pilot emulator is sim-only; real pilot arms/switches by RC'}
        a.output.write_text(json.dumps(report, indent=1))
        say(f'finished {state} {reason}')
        raise SystemExit(0)

    def sticks():
        # What the pilot's hands do during the whole procedure: throttle fully down, other sticks centred.
        m = ManualControlSetpoint()
        now_us = int(sim_now()*1e6)
        m.timestamp = m.timestamp_sample = max(0, now_us-20000)    # PX4 drops samples stamped in its future
        m.valid, m.data_source = True, ManualControlSetpoint.SOURCE_MAVLINK_0
        m.roll = m.pitch = m.yaw = 0.
        m.throttle = -1.
        m.flaps = m.aux1 = m.aux2 = m.aux3 = m.aux4 = m.aux5 = m.aux6 = 0.
        m.sticks_moving, m.buttons = False, 0
        sticks_pub.publish(m)
        st['sticks_sent'] += 1

    def tick():
        s = st['state']
        wall = time.monotonic()
        snapshot()
        if wall-st['started_wall'] > a.max_wall_s:
            if st['armed_at'] is not None and not st['land_sent']:
                set_mode(4, 6); st['land_sent'] = True; event('request_land', by='pilot', reason='wall deadline')
            finish('FAULT', 'wall deadline')
        if s is None:
            return
        fresh_sp = st['last_sp_wall'] is not None and wall-st['last_sp_wall'] < 1.
        if st['phase'] == 'WAIT_READY':
            ok = (s.get('px4_link') and s.get('local_enu') is not None and s.get('odom_enu') is not None
                  and fresh_sp and not s.get('armed') and s.get('landed') is not False)
            if ok:
                st['ready_since'] = st['ready_since'] or wall
                if wall-st['ready_since'] >= a.ready_hold_s:
                    event('arming_attempt', mode='ALTCTL')
                    set_mode(2)            # Altitude (ALTCTL) first, as the pilot arms on Altitude
                    arm()
                    st['phase'], st['arm_cmd_wall'], st['arm_first_wall'] = 'ARMING', wall, wall
            else:
                st['ready_since'] = None
        elif st['phase'] == 'ARMING':
            if s.get('armed'):
                st['armed_at'] = sim_now(); event('armed_observed')
                st['phase'], st['pos_wall'] = 'SWITCH', wall
            elif wall-st['arm_first_wall'] > a.arm_timeout_s:
                event('arming_refused', wall_s=wall-st['arm_first_wall'])
                finish('FAULT', f'PX4 refused arming for {a.arm_timeout_s:g} s; see diagnostics')
            elif wall-st['arm_cmd_wall'] > 1.:
                arm(); st['arm_cmd_wall'] = wall
        elif st['phase'] == 'SWITCH':
            if wall-st['pos_wall'] >= 1.:
                set_mode(3)                # Position: the go signal for auto offboard
                event('switch_to_position')
                st['phase'] = 'FLYING'
        elif st['phase'] == 'FLYING':
            if s.get('offboard_active') and not st['offboard_seen']:
                st['offboard_seen'] = True; event('navigation_started', note='offboard active (team node flies)')
            if not s.get('armed'):
                event('landed_disarmed', disarmed_source_sim_s=sim_now())
                finish('COMPLETE' if st['offboard_seen'] and not st['land_sent'] else 'ABORT_LANDED',
                       None if not st['land_sent'] else 'pilot landed the vehicle')
            took_back = st['offboard_seen'] and not s.get('offboard_active')
            overdue = sim_now()-st['armed_at'] > a.max_flight_s
            never = not st['offboard_seen'] and sim_now()-st['armed_at'] > 12.
            if (took_back or overdue or never) and not st['land_sent'] and not s.get('landed'):
                set_mode(4, 6)             # AUTO.LAND
                st['land_sent'] = True
                event('request_land', by='pilot', reason='offboard ended' if took_back else ('overdue' if overdue else 'no offboard'))

    node.create_timer(.1, tick)
    if sticks_pub is not None:
        node.create_timer(.05, sticks)     # 20 Hz sim time; PX4 drops the source after COM_RC_LOSS_T (0.5 s)
        say('SIM ONLY: publishing the pilot transmitter state (throttle down, sticks centred) as manual_control_input')
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        finish('FAULT', 'interrupted')
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
