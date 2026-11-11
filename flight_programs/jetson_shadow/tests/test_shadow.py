#!/usr/bin/env python3
"""Tests for upload_to_flyx_v4_skybeta.py shadow mode (python3 -m unittest -v test_shadow).

Pure (no ROS): default is shadow, forbidden-topic guard, embedded sources == v4c, the fake-armed state
machine (STREAM -> WAIT_OFFBOARD -> WAIT_ARM -> TAKEOFF, stays there on the bench; NAVIGATE + one real
planning step with --fake-altitude), refusal while the REAL vehicle is armed, fake land.
ROS (needs rclpy; skipped otherwise): the running program's publishers (graph API) never include
/skybeta/setpoint/* or /fmu/in/*, with a mock drone; with the real state armed it logs loudly,
never fake-arms, and publishes no setpoints at all.
"""
import importlib.util
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROGRAM = HERE.parent / "upload_to_flyx_v4_skybeta.py"
MOCK = HERE / "mock_drone.py"

_spec = importlib.util.spec_from_file_location("v4sky", PROGRAM)
v4 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(v4)          # executes the embedded modules; rclpy is imported only in main()


def has_rclpy():
    try:
        import rclpy  # noqa: F401
        return True
    except ImportError:
        return False


class PurePlanner:
    """The real embedded OnlinePlanner in-process (PlannerProcess forks; not needed here)."""
    def __init__(self):
        self.p, *_ = v4._build_planner_embedded()
        self.pad_xy = [float(v) for v in self.p.checkpoints[-1].centre]
        self.events, self.cleared = self.p.events, self.p.cleared

    def update(self, xy, yaw):
        cmd = self.p.update(xy, yaw)
        self.events, self.cleared = self.p.events, self.p.cleared
        return cmd


def run_machine(seconds, real_armed=False, fake_altitude=False, fake=True, plan=False):
    """Drive Mission + ShadowPilot exactly like the node's control loop (20 Hz, fresh inputs)."""
    planner = PurePlanner()
    pilot = v4.ShadowPilot(1.0, fake_altitude, 3.0, lift_m=.9 - .05) if fake else None
    mission = v4.Mission(planner, v4.MissionConfig(execute=fake, altitude=.9, carrot_m=.5,
                                                   pad_xy=tuple(planner.pad_xy), ack_timeout_s=1e9))
    tel = v4.Telemetry(now=0.0)
    tel.pose_frame = v4.POSE_FRAME_NED
    real = {"armed": real_armed, "offboard_active": real_armed, "landed": not real_armed}
    enu, yaw = (0.12, -0.05, 0.05), 0.3
    q = v4.enu_flu_to_ned_frd((v4.math.cos(yaw / 2), 0.0, 0.0, v4.math.sin(yaw / 2)))
    states, plans = [], []
    for i in range(int(seconds * 20)):
        now = i * .05
        lift = pilot.lift_m if pilot else 0.0
        tel.odom_t, tel.position_ned, tel.q_ned_frd = now, v4.enu_to_ned((enu[0], enu[1], enu[2] + lift)), q
        tel.status_t = tel.local_t = tel.landed_t = now
        tel.xy_valid = tel.z_valid = True
        tel.now = now
        v4.vehicle_view(tel, real, pilot, mission.state, fly=False)
        out = mission.tick(tel)
        if pilot:
            pilot.step(out.state, now, out.command)
        if not states or states[-1] != out.state:
            states.append(out.state)
        if plan and out.state == "NAVIGATE" and i % 4 == 0 and len(plans) < 3:
            plans.append(mission.plan())
    return states, mission, pilot, plans


class PureTests(unittest.TestCase):
    def test_default_is_shadow(self):
        self.assertFalse(v4.parse_args([]).execute)
        self.assertFalse(v4.parse_args(["--shadow"]).execute)
        self.assertTrue(v4.parse_args(["--fly"]).execute)
        self.assertTrue(v4.parse_args([]).fake_arm)
        self.assertEqual(v4.parse_args([]).depth_topic, "/camera/depth/image_raw")

    def test_forbidden_topics(self):
        f = v4.is_forbidden_topic
        for t in ("/skybeta/setpoint/pose", "/skybeta/setpoint/twist", "skybeta/setpoint/pose",
                  "/fmu/in/trajectory_setpoint", "/fmu/in/vehicle_command", "/fmu/in/offboard_control_mode"):
            self.assertTrue(f(t), t)
        for t in ("/jolie_flyx/shadow/setpoint_pose", "/jolie_flyx/status", "/fmu/out/vehicle_odometry"):
            self.assertFalse(f(t), t)
        self.assertTrue(f("/fmu/in/vehicle_command", shadow=False))      # never, even when flying
        self.assertFalse(f("/skybeta/setpoint/pose", shadow=False))      # --fly may

    def test_shadow_topic_cannot_be_a_vehicle_topic(self):
        with self.assertRaises(SystemExit):
            v4.main(["--shadow-topic", "/skybeta/setpoint/pose"])
        with self.assertRaises(SystemExit):
            v4.main(["--fly", "--fake-altitude"])

    def test_embedded_sources_are_v4c(self):
        import hashlib
        src = HERE.parent / "build_v4_skybeta.py"
        spec = importlib.util.spec_from_file_location("b", src)
        b = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(b)
        except Exception as e:                 # jolie_course/ not next to it (copied folder)
            self.skipTest(f"builder not runnable here: {e}")
        flown = os.environ.get("V4C_SOURCE")
        if not flown:
            self.skipTest("set V4C_SOURCE to the v4c file to compare")
        self.assertEqual(hashlib.sha256(Path(flown).read_bytes().replace(b"\r\n", b"\n")).hexdigest(), b.SOURCE_SHA256)
        self.assertEqual(b.sources_dict(PROGRAM.read_text()), b.sources_dict(Path(flown).read_text()))

    def test_fake_armed_progresses_and_waits_in_takeoff(self):
        states, mission, pilot, _ = run_machine(25)
        self.assertEqual(states, ["WAIT_HEALTH", "STREAM", "WAIT_OFFBOARD", "WAIT_ARM", "TAKEOFF"])
        self.assertTrue(pilot.armed and pilot.offboard)
        self.assertEqual(pilot.lift_m, 0.0)

    def test_fake_altitude_reaches_navigate_and_plans(self):
        states, mission, pilot, plans = run_machine(15, fake_altitude=True, plan=True)
        self.assertEqual(states[-1], "NAVIGATE", states)
        self.assertAlmostEqual(pilot.lift_m, .85)
        self.assertTrue(plans and all(p is not None for p in plans), plans)
        self.assertIn(plans[-1].mode, ("move", "turn", "done", "failed"))
        self.assertIsNotNone(mission.target)

    def test_refuses_while_real_vehicle_armed(self):
        states, mission, pilot, _ = run_machine(10, real_armed=True)
        self.assertEqual(states, ["WAIT_HEALTH"])
        self.assertFalse(pilot.armed)

    def test_plan_only_shadow(self):
        states, *_ = run_machine(5, fake=False)
        self.assertEqual(states, ["WAIT_HEALTH", "SHADOW"])

    def test_fake_land_disarms(self):
        pilot = v4.ShadowPilot(1.0)
        pilot.step("WAIT_OFFBOARD", 0.0)
        pilot.step("WAIT_OFFBOARD", 1.0)
        self.assertTrue(pilot.armed)
        pilot.step("LAND", 2.0)
        pilot.step("LAND", 4.1)
        self.assertFalse(pilot.armed)


@unittest.skipUnless(has_rclpy(), "rclpy not available")
class RosTests(unittest.TestCase):
    """Program + mock drone as subprocesses; graph inspected by node name."""

    def start(self, args, log):
        return subprocess.Popen([sys.executable, "-u", *args], stdout=log, stderr=subprocess.STDOUT,
                                env={**os.environ, "RCUTILS_COLORIZED_OUTPUT": "0"})

    def stop(self, p):
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:
                p.wait(20)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        return p.returncode

    def graph_publishers(self, node_name, seconds):
        import rclpy
        from rclpy.node import Node
        rclpy.init()
        n = Node("test_graph_probe")
        seen, setpoint_pubs = set(), set()
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < seconds:
                rclpy.spin_once(n, timeout_sec=.2)
                for name, ns in n.get_node_names_and_namespaces():
                    if name == node_name:
                        seen |= {t for t, _ in n.get_publisher_names_and_types_by_node(name, ns)}
                for t in ("/skybeta/setpoint/pose", "/skybeta/setpoint/twist"):
                    setpoint_pubs |= {i.node_name for i in n.get_publishers_info_by_topic(t)}
        finally:
            n.destroy_node()
            rclpy.try_shutdown()
        return seen, setpoint_pubs

    def run_case(self, mock_args, program_args, seconds=25):
        tmp = Path(tempfile.mkdtemp())
        mlog, plog = open(tmp / "mock.log", "w"), open(tmp / "program.log", "w")
        mock = self.start([str(MOCK), *mock_args], mlog)
        prog = self.start([str(PROGRAM), "--output", str(tmp / "out"), *program_args], plog)
        try:
            time.sleep(8)
            pubs, sp_nodes = self.graph_publishers("jolie_flyx", seconds - 8)
        finally:
            rc = self.stop(prog)
            self.stop(mock)
            mlog.close()
            plog.close()
        text = (tmp / "program.log").read_text()
        journal = [json.loads(line) for line in (tmp / "out" / "journal.jsonl").read_text().splitlines()]
        return pubs, sp_nodes, text, journal, rc

    def test_shadow_never_publishes_vehicle_topics(self):
        pubs, sp_nodes, text, journal, rc = self.run_case([], ["--fake-altitude"], seconds=40)
        self.assertIn("/jolie_flyx/status", pubs, text)
        self.assertIn("/jolie_flyx/shadow/setpoint_pose", pubs)
        self.assertFalse([t for t in pubs if t.startswith("/fmu/in") or t.startswith("/skybeta/setpoint")], pubs)
        self.assertNotIn("jolie_flyx", sp_nodes)
        self.assertIn("square_mission", sp_nodes)                 # the foreign one is seen ...
        self.assertIn("foreign publisher(s) on /skybeta/setpoint/pose", text)   # ... and tolerated
        self.assertNotIn("Traceback", text)
        self.assertEqual(rc, 0, text[-2000:])
        states = [r["state"] for r in journal if r["kind"] == "state"]
        want = ["WAIT_HEALTH", "STREAM", "WAIT_OFFBOARD", "WAIT_ARM", "TAKEOFF"]
        # a host pause in STREAM may FAIL the mission once; shadow then restarts it (FAILED -> WAIT_HEALTH)
        self.assertTrue(any(states[i:i + 5] == want for i in range(len(states))), states)
        for i, s in enumerate(states[:-1]):
            if s == "FAILED":
                self.assertEqual(states[i + 1], "WAIT_HEALTH", states)
        self.assertIn("NAVIGATE", states)
        end = [r for r in journal if r["kind"] == "end"][0]
        self.assertGreater(end["shadow_setpoints"], 100)
        self.assertFalse(end["violation"])

    def test_real_armed_logs_loudly_and_stays_silent(self):
        pubs, sp_nodes, text, journal, rc = self.run_case(["--armed", "--offboard"], [], seconds=15)
        self.assertIn("REAL VEHICLE armed=True", text)
        self.assertNotIn("FAKE ARM", text)
        self.assertNotIn("jolie_flyx", sp_nodes)
        self.assertFalse([t for t in pubs if t.startswith("/fmu/in") or t.startswith("/skybeta/setpoint")], pubs)
        end = [r for r in journal if r["kind"] == "end"][0]
        self.assertEqual(end["shadow_setpoints"], 0)
        self.assertTrue(end["real_armed_seen"])
        self.assertEqual(end["state"], "WAIT_HEALTH")


if __name__ == "__main__":
    unittest.main(verbosity=2)
