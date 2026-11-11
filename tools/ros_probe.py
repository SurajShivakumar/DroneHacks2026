"""Observe discovered ROS/PX4/sensor streams without publishing or commanding.

Run in the local ROS environment with matching px4_msgs sourced. It reports
callback timing separately from source stamp progression; neither establishes
estimator accuracy or permission to fly. No subscriber value is used as truth.
"""

import argparse
import json
from pathlib import Path
import sys
import time
import re

# Permit invocation by absolute script path from an external WSL runtime.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dronebench.telemetry import StreamStats, json_safe
from dronebench.vio_quality import finite_xyz_count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-disarmed", action="store_true")
    parser.add_argument("--snapshot-dir", type=Path)
    parser.add_argument("--record-topic", action="append")
    parser.add_argument("--record-dir", type=Path, action="append")
    parser.add_argument("--record-odometry-topic", action="append")
    parser.add_argument("--odometry-output", type=Path)
    parser.add_argument("--record-feature-topic", action="append")
    parser.add_argument("--feature-output", type=Path)
    parser.add_argument('--stop-file',type=Path,help='Operator recorder stop sentinel; never a flight command')
    parser.add_argument('--record-depth-topic')
    parser.add_argument('--depth-dir',type=Path)
    parser.add_argument('--depth-units',choices=['metres','millimetres'],default='metres')
    parser.add_argument('--depth-provenance')
    args = parser.parse_args()
    depth_sequence=None
    if args.record_depth_topic or args.depth_dir:
        if not (args.record_depth_topic and args.depth_dir and args.depth_provenance):
            parser.error('depth recording requires explicit topic, directory and provenance')
        from dronebench.depth_recording import DepthSequence
        depth_sequence=DepthSequence(args.depth_dir,topic=args.record_depth_topic,provenance=args.depth_provenance)
    record_topics=args.record_topic or []
    record_dirs=args.record_dir or []
    if len(record_topics)!=len(record_dirs) or len(set(record_topics))!=len(record_topics):
        parser.error("provide one unique record-topic for each record-dir")
    odometry_topics=args.record_odometry_topic or []
    if bool(odometry_topics)!=bool(args.odometry_output) or len(set(odometry_topics))!=len(odometry_topics):
        parser.error("record-odometry-topic requires one unique topic and odometry-output")
    feature_topics=args.record_feature_topic or []
    if bool(feature_topics)!=bool(args.feature_output) or len(set(feature_topics))!=len(feature_topics):
        parser.error("record-feature-topic requires unique topics and feature-output")
    sequences = {}
    if record_dirs:
        from dronebench.recording import ImageSequence
        from hashlib import sha256
        for topic,directory in zip(record_topics,record_dirs):
            sequences[topic] = ImageSequence(directory, topic=topic,
                clock_domain="ROS simulation time",
                provenance={"probe_sha256": sha256(Path(__file__).read_bytes()).hexdigest(),
                            "kind": "live_read_only_sensor_capture_not_flight"})
    if not 30 <= args.seconds <= 1200:
        parser.error("observation duration must be between 30 and 1200 wall seconds")
    # Fail before connecting to ROS if evidence would be overwritten.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.snapshot_dir:
        args.snapshot_dir.mkdir(parents=True, exist_ok=False)
    with args.output.open("x", encoding="utf-8") as output:
        odometry_stream=(args.odometry_output.open("x",encoding="utf-8")
                         if args.odometry_output else None)
        odometry_counts={topic:0 for topic in odometry_topics}
        feature_stream=(args.feature_output.open("x",encoding="utf-8") if args.feature_output else None)
        feature_counts={topic:0 for topic in feature_topics}
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rosidl_runtime_py.utilities import get_message

        rclpy.init()
        node = rclpy.create_node("dronehacks_read_only_probe")
        node.set_parameters([Parameter("use_sim_time", value=True)])
        streams, subscriptions, errors = {}, {}, []
        captured = set()
        last_images = {}
        allowed_types = {
            "px4_msgs/msg/VehicleStatus", "px4_msgs/msg/VehicleLocalPosition",
            "px4_msgs/msg/VehicleLandDetected", "px4_msgs/msg/EstimatorStatusFlags",
            "px4_msgs/msg/VehicleOdometry", "sensor_msgs/msg/Image",
            "sensor_msgs/msg/CameraInfo", "sensor_msgs/msg/Imu",
            "sensor_msgs/msg/Range", "sensor_msgs/msg/LaserScan",
            "nav_msgs/msg/Odometry", "sensor_msgs/msg/PointCloud2", "rosgraph_msgs/msg/Clock",
        }

        def save_snapshot(topic, message, suffix=""):
            from sim.snapshots import decode_image
            from PIL import Image
            import numpy as np
            array = decode_image(message)
            name = re.sub(r"[^A-Za-z0-9_-]", "_", topic) + suffix
            if message.encoding.lower() in ("32fc1", "16uc1"):
                np.save(args.snapshot_dir / f"{name}.npy", array)
            else:
                Image.fromarray(array).save(args.snapshot_dir / f"{name}.png")

        def callback(topic, message):
            if depth_sequence is not None and topic==args.record_depth_topic:
                from tools.ros_depth_mapper import decode_depth
                source_ns=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
                depth_sequence.append(decode_depth(message,args.depth_units),source_ns,message.header.frame_id)
            if args.require_disarmed and hasattr(message, "arming_state"):
                if message.arming_state == message.ARMING_STATE_ARMED:
                    raise RuntimeError("armed state observed during disarmed-only smoke test")
            # Avoid converting multi-megabyte image payloads just to discard them.
            feature_fields = None
            if topic in feature_topics:
                count = finite_xyz_count(message)
                stamp_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
                sample = {"topic": topic, "source_s": stamp_ns/1e9, "finite_xyz_count": count,
                          "reported_width": message.width, "reported_height": message.height}
                feature_stream.write(json.dumps(sample,allow_nan=False)+"\n")
                feature_stream.flush(); feature_counts[topic] += 1
                feature_fields = {"width":message.width,"height":message.height,
                                  "finite_xyz_count":count,"header":message_to_ordereddict(message.header)}
            if hasattr(message, "encoding") and hasattr(message, "height"):
                if topic in sequences:
                    from sim.snapshots import decode_image
                    from PIL import Image
                    stamp_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
                    sequence=sequences[topic]
                    incoming = sequence.output / "incoming.png"
                    Image.fromarray(decode_image(message)).save(incoming)
                    sequence.append(incoming, stamp_ns)
                    incoming.unlink()
                fields = {name: getattr(message, name) for name in
                          ("height", "width", "encoding", "step", "is_bigendian")}
                fields["header"] = message_to_ordereddict(message.header)
                if args.snapshot_dir:
                    last_images[topic] = message
                    if topic not in captured:
                        save_snapshot(topic, message)
                        captured.add(topic)
            elif feature_fields is not None:
                fields = feature_fields
            else:
                fields = message_to_ordereddict(message)
            source = None
            if "timestamp" in fields:
                source = fields["timestamp"] / 1e6
            elif "header" in fields:
                stamp = fields["header"]["stamp"]
                source = stamp["sec"] + stamp["nanosec"] / 1e9
            elif "clock" in fields:
                source = fields["clock"]["sec"] + fields["clock"]["nanosec"] / 1e9
            streams[topic]["stats"].observe(time.monotonic(), source, fields)
            if topic in odometry_counts:
                if source is None or "pose" not in fields or "twist" not in fields:
                    raise ValueError("recorded odometry must have a source stamp, pose and twist")
                pose=fields["pose"]["pose"]
                twist=fields["twist"]["twist"]
                sample={"topic":topic,"source_s":source,
                    "frame_id":fields["header"]["frame_id"],"child_frame_id":fields["child_frame_id"],
                    "position":pose["position"],"orientation":pose["orientation"],
                    "pose_covariance":fields["pose"]["covariance"],
                    "linear_velocity":twist["linear"],"angular_velocity":twist["angular"],
                    "twist_covariance":fields["twist"]["covariance"]}
                odometry_stream.write(json.dumps(json_safe(sample),allow_nan=False)+"\n")
                odometry_stream.flush();odometry_counts[topic]+=1

        started = time.monotonic()
        next_discovery = started
        try:
            while time.monotonic() - started < args.seconds and not (args.stop_file and args.stop_file.exists()):
                if time.monotonic() >= next_discovery:
                    for topic, types in node.get_topic_names_and_types():
                        if topic in subscriptions:
                            continue
                        choices = [t for t in types if t in allowed_types]
                        if len(choices) != 1:
                            continue
                        try:
                            cls = get_message(choices[0])
                            publishers = node.get_publishers_info_by_topic(topic)
                            offered = [info.qos_profile.reliability for info in publishers]
                            reliable_image = (choices[0] == "sensor_msgs/msg/Image" and bool(offered)
                                              and all(policy == ReliabilityPolicy.RELIABLE for policy in offered))
                            qos = (QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
                                   if reliable_image else qos_profile_sensor_data)
                            streams[topic] = {"type": choices[0], "stats": StreamStats(),
                                              "subscriber_reliability": qos.reliability.name,
                                              "publisher_reliability": [policy.name for policy in offered]}
                            subscriptions[topic] = node.create_subscription(
                                cls, topic, lambda msg, name=topic: callback(name, msg),
                                qos)
                        except (ImportError, AttributeError, ModuleNotFoundError) as exc:
                            errors.append({"topic": topic, "error": str(exc)})
                    next_discovery = time.monotonic() + 2
                rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if args.snapshot_dir:
                for topic, message in last_images.items():
                    save_snapshot(topic, message, "_latest")
            report = {"kind": "read_only_ros_observation_not_flight_qualification",
                      "duration_wall_s": now - started, "published_commands": False,
                      "recordings": {topic:{"frames":len(sequence.frames),
                                             "finalized":bool(sequence.frames)}
                                     for topic,sequence in sequences.items()},
                      "odometry_recordings": {"topics":odometry_counts,
                                              "file":str(args.odometry_output) if args.odometry_output else None},
                      "depth_recording": {"topic":args.record_depth_topic,
                          "frames":len(depth_sequence.frames) if depth_sequence else 0,
                          "directory":str(args.depth_dir) if args.depth_dir else None},
                      "feature_recordings": {"topics":feature_counts,
                                             "file":str(args.feature_output) if args.feature_output else None},
                      "streams": {topic: {"type": value["type"],
                                          "subscriber_reliability": value["subscriber_reliability"],
                                          "publisher_reliability": value["publisher_reliability"],
                                          **value["stats"].summary(now)}
                                  for topic, value in streams.items()}, "errors": errors}
            json.dump(json_safe(report), output, indent=2, allow_nan=False)
            output.write("\n")
            print(args.output)
        finally:
            for sequence in sequences.values():
                if sequence.frames:
                    sequence.close()
            if odometry_stream:
                odometry_stream.close()
            if feature_stream:
                feature_stream.close()
            if depth_sequence is not None:
                depth_sequence.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
