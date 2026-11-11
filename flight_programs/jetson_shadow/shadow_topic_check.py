#!/usr/bin/env python3
"""Read-only ROS 2 topic check for the shadow test (publishes nothing, calls no service).

    python3 shadow_topic_check.py                       # 60 s, report on stdout, exit 0 = PASS
    python3 shadow_topic_check.py --duration 60 --json report.json
    python3 shadow_topic_check.py --preflight           # one /skybeta/state: exit 0 landed+disarmed, 2 armed, 3 none

Per topic: messages, mean rate, max gap, last age -> PASS/FAIL against the expected rate. Gaps are
measured after a warm-up (default 5 s: the OAK depth stream is lazy and starts when subscribed).
Graph checks (node names): the program node (default jolie_flyx) must be up, must publish its
status and shadow topics, and must NOT publish /skybeta/setpoint/* or any /fmu/in/* topic.
Other publishers of those topics (the flight interface, the organizer's square example) are reported
as foreign, not failed. The real /skybeta/state reporting armed or offboard during the check FAILS it.
Humble / Python 3.10; numpy only for the depth valid-pixel statistic.
"""
import argparse
import json
import sys
import time

DEFAULT_PROGRAM = "jolie_flyx"
STREAMING = ("STREAM", "WAIT_OFFBOARD", "WAIT_ARM", "TAKEOFF", "NAVIGATE")

# key, topic, type, min mean Hz, max gap s, required
TOPICS = [
    ("odometry", "/visual_slam/tracking/odometry", "nav_msgs/Odometry", 20.0, .5, True),
    ("skybeta_state", "/skybeta/state", "std_msgs/String", 2.0, 1.0, True),
    ("depth", "/camera/depth/image_raw", "sensor_msgs/Image", 15.0, 1.0, True),
    ("depth_info", "/camera/depth/camera_info", "sensor_msgs/CameraInfo", 5.0, 1.5, True),
    ("imu", "/oak/imu/data", "sensor_msgs/Imu", 100.0, .25, True),
    ("sensor_combined", "/fmu/out/sensor_combined", "px4_msgs/SensorCombined", 50.0, .5, False),
    ("status", "/jolie_flyx/status", "std_msgs/String", 15.0, .5, True),
    ("shadow_setpoint", "/jolie_flyx/shadow/setpoint_pose", "geometry_msgs/PoseStamped", 15.0, .5, True),
]


def import_type(name):
    pkg, cls = name.split("/")
    mod = __import__(pkg + ".msg", fromlist=[cls])
    return getattr(mod, cls)


class Stat:
    def __init__(self, key, topic, type_name, min_hz, max_gap, required):
        self.key, self.topic, self.type_name = key, topic, type_name
        self.min_hz, self.max_gap_allowed, self.required = min_hz, max_gap, required
        self.times = []                  # arrival times (monotonic)
        self.walls = []                  # the same arrivals, wall clock (only for log correlation)
        self.skip = ""                   # reason the topic could not be subscribed
        self.extra = {}

    def evaluate(self, t_window, t_end, gap_ok=None):
        """Rate over the window, max gap (incl. window start -> first, last -> end), last age."""
        w = [t for t in self.times if t >= t_window]
        res = {"topic": self.topic, "type": self.type_name, "count": len(self.times), "count_window": len(w),
               "min_hz": self.min_hz, "max_gap_allowed_s": self.max_gap_allowed, **self.extra}
        if self.skip:
            res.update(verdict="SKIP" if not self.required else "FAIL", why=self.skip)
            return res
        span = t_end - t_window
        res["mean_hz"] = round(len(w) / span, 2) if span > 0 else 0.0
        res["last_age_s"] = round(t_end - self.times[-1], 3) if self.times else None
        edges = [t_window] + w + [t_end]
        gaps = [(b - a, a) for a, b in zip(edges, edges[1:])]
        if gap_ok is not None:           # only gaps that started while the topic should be flowing
            gaps = [(g, a) for g, a in gaps if gap_ok(a)]
        worst = max(gaps) if gaps else (0.0, t_window)
        res["max_gap_s"], res["max_gap_at_s"] = round(worst[0], 3), round(worst[1] - t_window, 1)
        # wall-clock start of each over-limit gap, from the arrival's own wall stamp (a VM can step the
        # wall clock, so a single monotonic->wall offset would misplace gaps)
        wall_of = dict(zip(self.times, self.walls))
        res["gaps_over"] = [[round(wall_of.get(a, a - time.monotonic() + time.time()), 2), round(g, 3)]
                            for g, a in gaps if g > self.max_gap_allowed]
        why = []
        if not w:
            why.append("no messages")
        elif res["mean_hz"] < self.min_hz:
            why.append(f"rate {res['mean_hz']} Hz < {self.min_hz}")
        if worst[0] > self.max_gap_allowed:
            why.append(f"gap {worst[0]:.2f} s > {self.max_gap_allowed}")
        res["verdict"] = "FAIL" if why else "PASS"
        res["why"] = "; ".join(why)
        return res


def node_label(info):
    return info.node_namespace.rstrip("/") + "/" + info.node_name


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--warmup", type=float, default=5.0)
    ap.add_argument("--program-node", default=DEFAULT_PROGRAM)
    ap.add_argument("--depth-topic", default="/camera/depth/image_raw")
    ap.add_argument("--camera-info-topic", default="")
    ap.add_argument("--shadow-topic", default="/jolie_flyx/shadow/setpoint_pose")
    ap.add_argument("--require-fmu", action="store_true", help="FAIL (not SKIP) if px4_msgs is not importable")
    ap.add_argument("--no-program", action="store_true", help="inputs only (no program running)")
    ap.add_argument("--json", default="", help="write the report here as JSON")
    ap.add_argument("--preflight", action="store_true", help="read one /skybeta/state and exit")
    ap.add_argument("--preflight-timeout", type=float, default=10.0)
    a, _ = ap.parse_known_args(argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = Node("shadow_topic_check")
    try:
        if a.preflight:
            return preflight(node, a)
        return check(node, a, qos_profile_sensor_data)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def preflight(node, a):
    import rclpy
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    got = []
    node.create_subscription(String, "/skybeta/state", lambda m: got.append(m.data), qos_profile_sensor_data)
    t0 = time.monotonic()
    while not got and time.monotonic() - t0 < a.preflight_timeout:
        rclpy.spin_once(node, timeout_sec=.1)
    others = [n for n, ns in node.get_node_names_and_namespaces() if n == a.program_node]
    if not got:
        print(json.dumps({"preflight": "NO_STATE", "why": f"no /skybeta/state in {a.preflight_timeout:.0f} s"}))
        return 3
    try:
        st = json.loads(got[-1])
    except ValueError:
        print(json.dumps({"preflight": "BAD_STATE", "raw": got[-1][:200]}))
        return 3
    view = {k: st.get(k) for k in ("armed", "landed", "offboard_active", "nav_state_name", "px4_link")}
    if st.get("armed") or st.get("offboard_active"):
        print(json.dumps({"preflight": "ARMED", "state": view}))
        return 2
    if st.get("landed") is False:
        print(json.dumps({"preflight": "NOT_LANDED", "state": view}))
        return 2
    if others:
        print(json.dumps({"preflight": "PROGRAM_ALREADY_RUNNING", "node": a.program_node, "state": view}))
        return 4
    print(json.dumps({"preflight": "OK", "state": view}))
    return 0


def check(node, a, sensor_qos):
    import rclpy
    topics = [list(t) for t in TOPICS]
    for t in topics:
        if t[0] == "depth":
            t[1] = a.depth_topic
        elif t[0] == "depth_info":
            t[1] = a.camera_info_topic or a.depth_topic.rsplit("/", 1)[0] + "/camera_info"
        elif t[0] == "shadow_setpoint":
            t[1] = a.shadow_topic
        elif t[0] == "sensor_combined" and a.require_fmu:
            t[5] = True
    if a.no_program:
        topics = [t for t in topics if t[0] not in ("status", "shadow_setpoint")]
    stats = {t[0]: Stat(*t) for t in topics}
    status_log = []                      # (t, state) from the program's status
    real_armed = []                      # (t, view) whenever /skybeta/state says armed / offboard
    depth_valid = []

    def make_cb(st):
        def cb(msg):
            t = time.monotonic()
            st.times.append(t)
            st.walls.append(time.time())
            if st.key == "status":
                try:
                    status_log.append((t, json.loads(msg.data).get("state")))
                except ValueError:
                    pass
            elif st.key == "skybeta_state":
                try:
                    s = json.loads(msg.data)
                except ValueError:
                    return
                if s.get("armed") or s.get("offboard_active"):
                    real_armed.append((t, {k: s.get(k) for k in ("armed", "offboard_active", "nav_state_name")}))
            elif st.key == "depth" and len(st.times) % 10 == 1:
                st.extra.update(encoding=msg.encoding, width=msg.width, height=msg.height)
                if msg.encoding in ("16UC1", "mono16"):
                    try:
                        import numpy as np
                        d = np.frombuffer(bytes(msg.data), np.uint16)
                        depth_valid.append(float((d > 0).mean()))
                    except Exception:
                        pass
        return cb

    for st in stats.values():
        try:
            node.create_subscription(import_type(st.type_name), st.topic, make_cb(st), sensor_qos)
        except Exception as e:           # px4_msgs missing in this environment
            st.skip = f"cannot subscribe ({type(e).__name__}: {e})"

    graph_samples = []

    def sample_graph(t):
        out = {"t": t, "program_up": False, "program_publishes": [], "violations": [], "foreign": {}}
        try:
            names = node.get_node_names_and_namespaces()
        except Exception:
            names = []
        prog = [(n, ns) for n, ns in names if n == a.program_node]
        out["program_up"] = bool(prog)
        out["program_instances"] = len(prog)
        for n, ns in prog:
            try:
                pubs = [tn for tn, _ in node.get_publisher_names_and_types_by_node(n, ns)]
            except Exception:
                pubs = []
            out["program_publishes"] = sorted(set(out["program_publishes"]) | set(pubs))
        forbidden = ["/skybeta/setpoint/pose", "/skybeta/setpoint/twist"]
        forbidden += [tn for tn, _ in node.get_topic_names_and_types() if tn.startswith("/fmu/in")]
        for tn in out["program_publishes"]:
            if tn.startswith("/fmu/in") or tn.startswith("/skybeta/setpoint"):
                out["violations"].append(tn)
        for tn in forbidden:
            try:
                infos = node.get_publishers_info_by_topic(tn)
            except Exception:
                continue
            for i in infos:
                if i.node_name == a.program_node:
                    out["violations"].append(f"{tn} <- {node_label(i)}")
                else:
                    out["foreign"].setdefault(tn, set()).add(node_label(i))
        out["violations"] = sorted(set(out["violations"]))
        graph_samples.append(out)

    t0 = time.monotonic()
    t_window, t_end = t0 + a.warmup, t0 + a.warmup + a.duration
    next_graph = t_window              # after the warm-up: DDS discovery of the program takes a moment
    print(f"shadow_topic_check: {a.warmup:.0f} s warm-up + {a.duration:.0f} s, program node '{a.program_node}'",
          flush=True)
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=.02)
        now = time.monotonic()
        if now >= next_graph:
            sample_graph(now)
            next_graph = now + 5.0
    sample_graph(time.monotonic())
    t_end = time.monotonic()

    # ------------------------------------------------------------- verdicts
    def streaming_at(t):
        state = None
        for ts, s in status_log:
            if ts > t:
                break
            state = s
        return state in STREAMING

    results = {}
    for key, st in stats.items():
        results[key] = st.evaluate(t_window, t_end, gap_ok=streaming_at if key == "shadow_setpoint" else None)
    if "shadow_setpoint" in results and not any(streaming_at(t) for t, _ in status_log if t >= t_window):
        r = results["shadow_setpoint"]
        r.update(verdict="FAIL", why="the program never reached a streaming state (STREAM..NAVIGATE)")
    if depth_valid:
        results["depth"]["valid_px_mean"] = round(sum(depth_valid) / len(depth_valid), 3)

    fails = [f"{k}: {r['why']}" for k, r in results.items() if r["verdict"] == "FAIL"]
    violations = sorted({v for g in graph_samples for v in g["violations"]})
    foreign = {}
    for g in graph_samples:
        for tn, ns in g["foreign"].items():
            foreign.setdefault(tn, set()).update(ns)
    graph = {"violations": violations, "foreign": {k: sorted(v) for k, v in foreign.items()}}
    if not a.no_program:
        down = [round(g["t"] - t0, 1) for g in graph_samples if not g["program_up"]]
        graph["program_down_at_s"] = down
        graph["program_publishes"] = sorted({p for g in graph_samples for p in g["program_publishes"]})
        graph["program_instances_max"] = max((g.get("program_instances", 0) for g in graph_samples), default=0)
        if down:
            fails.append(f"program node '{a.program_node}' not in the graph at t = {down} s")
        if graph["program_instances_max"] > 1:
            fails.append(f"{graph['program_instances_max']} nodes named '{a.program_node}' (another copy running?)")
        for need in ("/jolie_flyx/status", a.shadow_topic):
            if need not in graph["program_publishes"]:
                fails.append(f"program does not publish {need}")
    if violations:
        fails.append("program publishes vehicle topics: " + ", ".join(violations))
    if real_armed:
        fails.append(f"REAL vehicle reported armed/offboard during the check ({len(real_armed)} msgs, "
                     f"first {real_armed[0][1]})")
    states = []
    for _, s in status_log:
        if not states or states[-1] != s:
            states.append(s)

    report = {"verdict": "FAIL" if fails else "PASS", "fails": fails, "duration_s": round(t_end - t_window, 1),
              "warmup_s": a.warmup, "topics": results, "graph": graph, "program_states": states,
              "real_armed_msgs": len(real_armed)}

    # ------------------------------------------------------------- print
    print(f"\n{'key':16} {'verdict':7} {'count':>6} {'mean Hz':>8} {'min Hz':>7} {'max gap':>8} {'allowed':>7} "
          f"{'last age':>8}  topic / why")
    for k, r in results.items():
        f = lambda v, fmt: "-" if v is None else format(v, fmt)
        print(f"{k:16} {r['verdict']:7} {r['count']:6d} {f(r.get('mean_hz'), '8.1f')} {r['min_hz']:7.1f} "
              f"{f(r.get('max_gap_s'), '8.2f')} {r['max_gap_allowed_s']:7.2f} {f(r.get('last_age_s'), '8.2f')}  "
              f"{r['topic']}" + (f"  <- {r['why']}" if r.get("why") else "")
              + (f"  (valid px {r['valid_px_mean']:.0%})" if "valid_px_mean" in r else ""))
    print(f"\nprogram states seen: {' -> '.join(str(s) for s in states) or 'none'}")
    if not a.no_program:
        print(f"program publishes: {', '.join(graph['program_publishes']) or 'nothing seen'}")
    print(f"vehicle-topic publishers from the program: {', '.join(violations) or 'NONE (good)'}")
    fmu_in = {}
    for tn, ns in graph["foreign"].items():
        if tn.startswith("/fmu/in"):
            for n in ns:
                fmu_in[n] = fmu_in.get(n, 0) + 1
        else:
            print(f"foreign publisher(s) on {tn}: {', '.join(ns)}  (not ours: reported, not failed)")
    for n, count in sorted(fmu_in.items()):
        print(f"foreign publisher on /fmu/in/* ({count} topics): {n}  (the flight interface owns /fmu/in)")
    for msg in fails:
        print(f"FAIL: {msg}")
    print(f"\nCHECK {report['verdict']}", flush=True)
    if a.json:
        with open(a.json, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=1, default=str)
    return 0 if not fails else 1


if __name__ == "__main__":
    sys.exit(main())
