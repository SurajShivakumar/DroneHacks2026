#!/usr/bin/env python3
"""Mock Sky Beta drone on the bench, for testing the shadow program off the Jetson (NOT for the drone).

Publishes, like skybeta-06: /visual_slam/tracking/odometry (stationary, 30 Hz), /skybeta/state (JSON,
10 Hz), /camera/depth/image_raw (16UC1 640x400 mm, 30 Hz, ~40 % valid noisy pixels, stamped 110 ms old)
+ camera_info (fx = fy = 398, cx 320, cy 200), /oak/imu/data (200 Hz), and a FOREIGN
/skybeta/setpoint/pose publisher (node 'square_mission', at the current pose, 10 Hz) like the
organizer's square example.

    python3 mock_drone.py [--armed] [--odom-gap-at S --odom-gap-s 3]
"""
import argparse
import array
import json
import signal
import subprocess
import sys
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String

POSE = (0.12, -0.05, 0.05)       # ENU odom, m (a little off the cuVSLAM origin)
YAW = 0.3                        # rad, ENU


class MockDrone(Node):
    def __init__(self, a, role):
        super().__init__("mock_skybeta_" + role)
        self.a, self.t0 = a, time.monotonic()
        self.odom_pub = self.create_publisher(Odometry, "/visual_slam/tracking/odometry", 10)
        self.state_pub = self.create_publisher(String, "/skybeta/state", 10)
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "/oak/imu/data", qos_profile_sensor_data)
        rng = np.random.default_rng(1)
        self.frames = []
        for _ in range(8):                               # noisy wall ~2.5 m, 40 % valid, flicker
            d = rng.normal(2500, 120, (400, 640)).clip(314, 9000).astype(np.uint16)
            d[rng.random((400, 640)) > .40] = 0
            self.frames.append(array.array('B', d.tobytes()))   # array: Humble's fast uint8[] path
        self.k = 0
        self.status_t0 = None               # first /jolie_flyx/status seen (gap timing relative to the program)
        if role == "pose":
            self.create_timer(1 / 30, self.on_odom)
            self.create_subscription(String, "/jolie_flyx/status", self.on_status, 10)
            self.create_timer(.1, self.on_state)
        elif role == "depth":
            self.create_timer(1 / 30, self.on_depth)
        elif role == "imu":
            self.create_timer(1 / 200, self.on_imu)
        self.gap_logged = False
        self.last_tick = None
        self.create_timer(.05, self.on_watch)

    def on_watch(self):
        """Log the mock's OWN pauses (host / VM hiccups), so a test gap can be blamed correctly."""
        now = time.monotonic()
        if self.last_tick is not None and now - self.last_tick > .15:
            self.get_logger().warn(f"MOCK PAUSE {now - self.last_tick:.2f} s at wall {time.time():.1f}")
        self.last_tick = now

    def on_status(self, msg):
        if self.status_t0 is None:
            self.status_t0 = time.monotonic()

    def on_odom(self):
        t = time.monotonic() - self.t0
        if self.a.odom_gap_after_status:            # gap at a fixed time after the program came up
            t = (time.monotonic() - self.status_t0) if self.status_t0 is not None else -1e9
        if self.a.odom_gap_at and self.a.odom_gap_at <= t < self.a.odom_gap_at + self.a.odom_gap_s:
            if not self.gap_logged:
                self.get_logger().info(f"ODOMETRY GAP {self.a.odom_gap_s} s starts (t = {t:.1f} s)")
                self.gap_logged = True
            return
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = "odom", "base_link"
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = POSE
        m.pose.pose.position.x += float(np.random.normal(0, .001))
        m.pose.pose.orientation.w, m.pose.pose.orientation.z = math.cos(YAW / 2), math.sin(YAW / 2)
        self.odom_pub.publish(m)

    def on_state(self):
        armed = bool(self.a.armed)
        st = {"armed": armed, "landed": not armed, "offboard_active": armed and self.a.offboard,
              "nav_state": 14 if armed and self.a.offboard else 2, "nav_state_name": "OFFBOARD" if armed and self.a.offboard else "POSCTL",
              "px4_link": True, "local_enu": list(POSE), "origin_enu": [0.0, 0.0, 0.0], "clamped": [],
              "envelope": {"z_min": 0.3}}
        self.state_pub.publish(String(data=json.dumps(st)))

    def on_depth(self):
        now = self.get_clock().now()
        stamp = (now - rclpy.duration.Duration(seconds=.11)).to_msg()
        im = Image()
        im.header.stamp, im.header.frame_id = stamp, "oak_right_camera_optical_frame"
        im.height, im.width, im.encoding, im.is_bigendian, im.step = 400, 640, "16UC1", 0, 1280
        im.data = self.frames[self.k % len(self.frames)]
        self.k += 1
        self.depth_pub.publish(im)
        ci = CameraInfo()
        ci.header = im.header
        ci.height, ci.width, ci.distortion_model = 400, 640, "plumb_bob"
        ci.k = [398.0, 0.0, 320.0, 0.0, 398.0, 200.0, 0.0, 0.0, 1.0]
        ci.p = [398.0, 0.0, 320.0, 0.0, 0.0, 398.0, 200.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(ci)

    def on_imu(self):
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "oak_imu_frame"
        m.linear_acceleration.z = 9.81
        self.imu_pub.publish(m)


class SquareMission(Node):
    """Foreign setpoint publisher, like the organizer's square example while disarmed."""
    def __init__(self):
        super().__init__("square_mission")
        self.pub = self.create_publisher(PoseStamped, "/skybeta/setpoint/pose", 10)
        self.create_timer(.1, self.on_timer)

    def on_timer(self):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = POSE
        m.pose.orientation.w = 1.0
        self.pub.publish(m)


ROLES = ("pose", "depth", "imu", "square")


def main():
    """One process per role (the GIL: one process could not keep 30 Hz images + 200 Hz IMU)."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--armed", action="store_true", help="state says armed (refusal test)")
    ap.add_argument("--offboard", action="store_true", help="with --armed: offboard_active too")
    ap.add_argument("--odom-gap-at", type=float, default=0.0, help="stop odometry at this time (s after start)")
    ap.add_argument("--odom-gap-s", type=float, default=3.0)
    ap.add_argument("--odom-gap-after-status", action="store_true",
                    help="--odom-gap-at counts from the first /jolie_flyx/status message, not from mock start")
    ap.add_argument("--role", default="", choices=("",) + ROLES)
    a, rest = ap.parse_known_args()
    if not a.role:
        def stop(signum, frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, stop)     # forward TERM/INT to the children (else they orphan)
        signal.signal(signal.SIGINT, stop)
        kids = [subprocess.Popen([sys.executable, "-u", __file__, *sys.argv[1:], "--role", r]) for r in ROLES]
        try:
            for k in kids:
                k.wait()
        except KeyboardInterrupt:
            pass
        finally:
            for k in kids:                      # our own children, by PID
                if k.poll() is None:
                    k.send_signal(signal.SIGTERM)
            for k in kids:
                try:
                    k.wait(5)
                except subprocess.TimeoutExpired:
                    k.kill()
        return
    rclpy.init()
    node = SquareMission() if a.role == "square" else MockDrone(a, a.role)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
