"""Sim launcher for the team's FlyX node (simready repo, pathfinding/flyx/pathplan_mission.py).

The node assumes cuVSLAM's ``odom`` frame: origin and heading = the rig pose at
SLAM start, with the drone on the launch mark facing up the first corridor. Our
sim's estimator (OpenVINS stand-in, relayed by tools/skybeta_vio_relay.py) is
gravity-up with an arbitrary heading and origin fixed at its own initialisation,
which happens on the bootstrap rig before the drone is set down.

So this launcher waits until the drone sits on the launch mark (it is started at
rig detach), averages the estimator's own /visual_slam/tracking/odometry pose
for a moment, and hands the node the equivalent ``launch_*`` parameters:

    heading h  = launch_heading_course - yaw0_odom
    launch_xy  = launch_mark - Rz(h) p0_odom_xy
    launch_z   = launch_height - p0_odom_z

so the node's OdomFrame maps that resting pose to the launch mark exactly as it
would on the drone. Only the estimator's output and the published launch mark are
used (no simulator truth). The node itself is run unmodified.
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path


def launch_params(p0, yaw0, mark_xy, mark_heading, mark_height):
    h = mark_heading-yaw0
    c, s = math.cos(h), math.sin(h)
    lx = mark_xy[0]-(c*p0[0]-s*p0[1])
    ly = mark_xy[1]-(s*p0[0]+c*p0[1])
    return [lx, ly], math.atan2(math.sin(h), math.cos(h)), mark_height-p0[2]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--node', type=Path, required=True, help='pathplan_mission.py (FlyX single file)')
    p.add_argument('--log', type=Path, required=True, help='JSON record of the computed frame parameters')
    p.add_argument('--launch-xy', type=float, nargs=2, default=(-2.515, .914))
    p.add_argument('--launch-heading', type=float, default=math.pi/2)
    p.add_argument('--launch-height', type=float, default=.05)
    p.add_argument('--settle-s', type=float, default=2., help='sim seconds of odometry averaged at rest')
    p.add_argument('--node-param', action='append', default=[], help='extra node parameter name:=value')
    a = p.parse_args()

    import rclpy
    from rclpy.parameter import Parameter
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = rclpy.create_node('pathplan_frame_probe')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    samples = []

    def on_odom(msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pp = msg.pose.pose.position
        samples.append((node.get_clock().now().nanoseconds*1e-9, pp.x, pp.y, pp.z, yaw))

    node.create_subscription(Odometry, '/visual_slam/tracking/odometry', on_odom, 10)
    t0 = time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=.1)
        if len(samples) > 5 and samples[-1][0]-samples[0][0] >= a.settle_s:
            break
        if time.monotonic()-t0 > 120:
            print('no /visual_slam/tracking/odometry for 120 s; launching with default frame', flush=True)
            break
    node.destroy_node()
    rclpy.shutdown()
    rec = {'samples': len(samples)}
    params = []
    if samples:
        n = len(samples)
        p0 = [sum(s[k] for s in samples)/n for k in (1, 2, 3)]
        yaw0 = math.atan2(sum(math.sin(s[4]) for s in samples), sum(math.cos(s[4]) for s in samples))
        lxy, lh, lz = launch_params(p0, yaw0, a.launch_xy, a.launch_heading, a.launch_height)
        rec.update(p0_odom=p0, yaw0_odom=yaw0, launch_xy_m=lxy, launch_heading_course_rad=lh, launch_height_m=lz,
                   spread_m=max(math.dist(s[1:4], p0) for s in samples))
        params = ['-p', f'launch_xy_m:=[{lxy[0]:.6f},{lxy[1]:.6f}]', '-p', f'launch_heading_course_rad:={lh:.6f}',
                  '-p', f'launch_height_m:={lz:.6f}']
    a.log.write_text(json.dumps(rec, indent=1))
    print('frame parameters', json.dumps(rec), flush=True)
    argv = [sys.executable, '-u', str(a.node), '--ros-args', '-p', 'use_sim_time:=true', *params,
            *[x for kv in a.node_param for x in ('-p', kv)]]
    print('exec', ' '.join(argv), flush=True)
    os.execv(sys.executable, argv)


if __name__ == '__main__':
    main()
