"""Team navigation node for the organizer Sky Beta stack (competitor interface only).

Inputs (real-drone topics):  /visual_slam/tracking/odometry (ENU 'odom', origin and
heading = startup pose), /skybeta/state (JSON from flight_interface).
Outputs: /skybeta/setpoint/pose (ENU position + yaw, persists) and, for landing,
/skybeta/setpoint/twist (ENU velocity, expires after 1 s). Never /fmu/in/*.

Flow (competitor guide §4): publish a setpoint every tick from startup (the
interface needs a fresh setpoint to allow Offboard); when `offboard_active`
turns true (pilot armed then switched to Position), take off immediately to the
reference's first point (PX4 disarms if not airborne 10 s after arming), follow
the course reference with a look-ahead carrot and a tracking clock, then descend
on the pad with a downward twist until disarmed.

Course frame: the published-map frame of the reference CSV. At Offboard start
the node anchors course <- odom from the known launch pose (launch x/y/yaw and
the base_link ground height), exactly what team_localization's CourseAnchor does.

Safety (this node's own, on top of the interface's envelope and the pilot):
cross-track error beyond --abort-m, stale odometry, or Offboard lost -> stop
advancing; abort/stale -> descend in place. The tracking clock slows/pauses the
reference on cross-track error (--pause-m) or along-track lag (--lag-pause-m).
"""
import argparse
import json
from math import atan2, cos, sin, pi
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dronebench.teammate_policy import ReferenceTrajectory  # noqa: E402


def wrap(a):
    return (a+pi) % (2*pi)-pi


def yaw_of(q):
    return atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Anchor:
    """course = R(dyaw) (odom - o0) + c0, with dyaw = launch_yaw - odom_yaw0."""
    def __init__(self, odom_xyz, odom_yaw, launch_xyz, launch_yaw):
        self.o0, self.c0 = np.asarray(odom_xyz, float), np.asarray(launch_xyz, float)
        self.dyaw = wrap(launch_yaw-odom_yaw)
        c, s = cos(self.dyaw), sin(self.dyaw)
        self.R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])

    def to_course(self, p):
        return self.R@(np.asarray(p, float)-self.o0)+self.c0

    def to_odom(self, c):
        return self.R.T@(np.asarray(c, float)-self.c0)+self.o0

    def yaw_to_odom(self, yaw_course):
        return wrap(yaw_course-self.dyaw)

    def vec_to_odom(self, v):
        return self.R.T@np.asarray(v, float)


class Clock:
    """Reference-time clock: full rate when on track, slows on cross-track error or lag."""
    def __init__(self, pause_m, lag_pause_m, slew=.5):
        self.pause, self.lag_pause, self.slew = pause_m, lag_pause_m, slew
        self.t, self.rate = 0., 0.

    def step(self, dt, cross_m, lag_m, floor_t=0.):
        target = max(0., min(1., (self.pause-cross_m)/(.6*self.pause), (self.lag_pause-lag_m)/(.6*self.lag_pause)))
        self.rate += max(-self.slew*dt, min(self.slew*dt, target-self.rate))
        self.t = max(self.t+dt*self.rate, floor_t)
        return self.t


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trajectory', type=Path, required=True, help='course-frame reference CSV')
    p.add_argument('--launch', type=float, nargs=3, default=(-2.515, .914, 1.5707963267948966),
                   metavar=('X', 'Y', 'YAW'), help='course pose of base_link on the launch mark')
    p.add_argument('--ground-z', type=float, default=.08, help='course z of base_link resting on the floor')
    p.add_argument('--lookahead-s', type=float, default=.3)
    p.add_argument('--pause-m', type=float, default=.05)
    p.add_argument('--lag-pause-m', type=float, default=.25)
    p.add_argument('--abort-m', type=float, default=.12)
    p.add_argument('--takeoff-tol-m', type=float, default=.06)
    p.add_argument('--descent-mps', type=float, default=.35)
    p.add_argument('--rate-hz', type=float, default=20.)
    p.add_argument('--journal', type=Path, required=True)
    a = p.parse_args()

    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped, TwistStamped

    ref = ReferenceTrajectory(a.trajectory, time_scale=1.)
    rclpy.init()
    node = rclpy.create_node('team_nav')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    pub_pose = node.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
    pub_twist = node.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
    reliable = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
    journal = a.journal.open('w')
    st = {'odom': None, 'odom_t': None, 'state': {}, 'phase': 'WAIT', 'anchor': None, 'clock': Clock(a.pause_m, a.lag_pause_m),
          'last_t': None, 'settle_since': None, 'phase_t': None, 'pad': None}

    def now():
        return node.get_clock().now().nanoseconds*1e-9

    def log(kind, **kw):
        journal.write(json.dumps({'kind': kind, 'sim_s': now(), **kw})+'\n')

    def phase(name, **kw):
        st['phase'], st['phase_t'] = name, now()
        log('phase', phase=name, **kw)
        node.get_logger().info(f'team_nav: {name} {kw}')

    def on_odom(m):
        pp, q = m.pose.pose.position, m.pose.pose.orientation
        st['odom'] = (np.array([pp.x, pp.y, pp.z]), yaw_of(q),
                      np.array([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z]))
        st['odom_t'] = now()

    def on_state(m):
        try:
            st['state'] = json.loads(m.data)
        except ValueError:
            pass

    def send_pose(p_odom, yaw_odom):
        m = PoseStamped()
        m.header.stamp, m.header.frame_id = node.get_clock().now().to_msg(), 'odom'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p_odom)
        m.pose.orientation.z, m.pose.orientation.w = sin(yaw_odom/2), cos(yaw_odom/2)
        pub_pose.publish(m)

    def send_twist(v_odom):
        m = TwistStamped()
        m.header.stamp, m.header.frame_id = node.get_clock().now().to_msg(), 'odom'
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = map(float, v_odom)
        pub_twist.publish(m)

    def start_landing(reason):
        if st['phase'] in ('LAND', 'DONE'):
            return
        st['pad'] = st['odom'][0].copy() if st['odom'] else None
        phase('LAND', reason=reason)

    def tick():
        t = now()
        odom, s = st['odom'], st['state']
        if odom is None:
            return
        p_od, yaw_od, _ = odom
        stale = st['odom_t'] is None or t-st['odom_t'] > .3
        active = bool(s.get('offboard_active'))
        if st['phase'] == 'WAIT':
            send_pose(p_od, yaw_od)               # keep the interface fed; it does not move the vehicle yet
            if active:
                st['anchor'] = Anchor(p_od, yaw_od, (a.launch[0], a.launch[1], a.ground_z), a.launch[2])
                phase('TAKEOFF', anchor_dyaw=st['anchor'].dyaw, odom0=p_od.tolist())
            return
        A = st['anchor']
        if st['phase'] in ('TAKEOFF', 'NAV') and (not active or stale):
            start_landing('offboard lost' if not active else 'stale odometry')
        if st['phase'] == 'TAKEOFF':
            target_c, _ = ref.sample(0.)
            yaw_c = ref.yaw(0.)
            send_pose(A.to_odom(target_c), A.yaw_to_odom(yaw_c))
            err = float(np.linalg.norm(A.to_course(p_od)-target_c))
            if err < a.takeoff_tol_m and float(np.linalg.norm(odom[2])) < .1:
                st['settle_since'] = st['settle_since'] or t
                if t-st['settle_since'] > .5:
                    st['last_t'] = t
                    phase('NAV', takeoff_s=t-st['phase_t'])
            else:
                st['settle_since'] = None
            log('tick', phase='TAKEOFF', err=err)
            return
        if st['phase'] == 'NAV':
            dt = max(0., t-st['last_t']); st['last_t'] = t
            c = A.to_course(p_od)
            clk = st['clock']
            near_t, _, cross = ref.nearest(c, around_s=clk.t)
            target, tangent = ref.sample(clk.t)
            tl = float(np.linalg.norm(tangent))
            lag = max(0., float((target-c)@tangent/tl)) if tl > 1e-6 else 0.
            if cross > a.abort_m:
                log('abort', cross=cross, ref_t=clk.t)
                start_landing(f'cross-track {cross:.3f} m > {a.abort_m}')
                return
            tr = clk.step(dt, cross, lag, floor_t=near_t)
            tc = min(ref.duration_s, tr+a.lookahead_s)
            carrot, _ = ref.sample(tc)
            send_pose(A.to_odom(carrot), A.yaw_to_odom(ref.yaw(tc)))
            log('tick', phase='NAV', ref_t=tr, rate=clk.rate, cross=cross, lag=lag, course=c.tolist())
            end, _ = ref.sample(ref.duration_s)
            if tr >= ref.duration_s-1e-6 and float(np.linalg.norm(c[:2]-end[:2])) < .06:
                log('reference_complete', nav_s=t-st['phase_t'])
                start_landing('reference complete')
            return
        if st['phase'] == 'LAND':
            pad = st['pad'] if st['pad'] is not None else p_od
            v = np.zeros(3)
            v[:2] = np.clip(1.2*(pad[:2]-p_od[:2]), -.2, .2)     # hold xy over the touchdown point
            v[2] = -a.descent_mps
            send_twist(v)
            if not s.get('armed', True):
                phase('DONE')
            return
        if st['phase'] == 'DONE':
            send_pose(p_od, yaw_od)

    node.create_subscription(Odometry, '/visual_slam/tracking/odometry', on_odom, reliable)
    node.create_subscription(String, '/skybeta/state', on_state, 10)
    node.create_timer(1./a.rate_hz, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception) as exc:
        log('exit', error=None if isinstance(exc, KeyboardInterrupt) else repr(exc))
    finally:
        journal.close()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
