"""Depth/CameraInfo/TF -> nav_msgs/OccupancyGrid; no flight command interface.

Requires rectified optical-Z depth with matching calibrated CameraInfo and a
time-aligned optical-to-ENU-map TF from a real estimator. Every output uses ONE
depth frame; historical free cells are not re-stamped as fresh. This deliberately
leaves sparse rays and unseen height voxels unknown. No live validation yet.
Run with --help for explicit sensor, map bounds and aircraft height envelope.
Consumers MUST expire grids by header stamp and reject pose resets/frame jumps.
"""

import argparse
from math import isfinite
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dronebench.depth_mapping import DepthMap, Pinhole


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def require_fresh(stamp, now, max_age, label):
    if stamp <= 0 or now <= 0 or not 0 <= now - stamp <= max_age * 1e9:
        raise ValueError(f"{label} missing, future or stale source timestamp")


def checked_camera(info, image, optical_frame, max_skew):
    if info.header.frame_id != optical_frame or image.header.frame_id != optical_frame:
        raise ValueError("depth and CameraInfo must share the configured optical frame")
    if (info.width, info.height) != (image.width, image.height):
        raise ValueError("depth/CameraInfo dimensions differ")
    if info.distortion_model not in ("plumb_bob", "rational_polynomial"):
        raise ValueError("unsupported distortion model; explicit rectified pinhole CameraInfo required")
    if stamp_ns(info.header.stamp) <= 0 or abs(stamp_ns(info.header.stamp) - stamp_ns(image.header.stamp)) > max_skew * 1e9:
        raise ValueError("CameraInfo not time-matched")
    k = np.asarray(info.k, dtype=float).reshape(3, 3)
    p = np.asarray(info.p, dtype=float).reshape(3, 4)
    r = np.asarray(info.r, dtype=float).reshape(3, 3)
    if (not np.isfinite(k).all() or not np.isfinite(p).all()
            or not np.isfinite(info.d).all() or any(abs(float(v)) > 1e-9 for v in info.d)
            or not np.allclose(r, np.eye(3), atol=1e-8, rtol=0)
            or not np.allclose(p[:, :3], k, atol=1e-5, rtol=0)
            or not np.allclose(p[:, 3], 0, atol=1e-8, rtol=0)
            or not np.allclose(k, [[k[0, 0], 0, k[0, 2]], [0, k[1, 1], k[1, 2]], [0, 0, 1]], atol=1e-8, rtol=0)):
        raise ValueError("requires zero-distortion rectified pinhole K/P/R; rectify/calibrate upstream")
    if info.binning_x not in (0, 1) or info.binning_y not in (0, 1) or any(
            getattr(info.roi, name) != 0 for name in ("x_offset", "y_offset", "width", "height")):
        raise ValueError("cropped/binned CameraInfo not supported")
    return Pinhole(int(info.width), int(info.height), k[0, 0], k[1, 1], k[0, 2], k[1, 2])


def decode_depth(image, units):
    formats = {"metres": ("32FC1", "f4", 1.), "millimetres": ("16UC1", "u2", .001)}
    encoding, dtype, scale = formats[units]
    if image.encoding.upper() != encoding:
        raise ValueError("encoding does not match explicitly selected depth units")
    dtype = np.dtype((">" if image.is_bigendian else "<") + dtype)
    row_bytes = image.width * dtype.itemsize
    if image.width < 1 or image.height < 1 or image.step < row_bytes or len(image.data) != image.height * image.step:
        raise ValueError("invalid depth dimensions, stride or payload")
    rows = np.frombuffer(image.data, dtype="u1").reshape(image.height, image.step)
    return np.ascontiguousarray(rows[:, :row_bytes]).view(dtype).reshape(image.height, image.width).astype(float) * scale


def transform_matrix(transform):
    q, t = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    if not all(isfinite(v) for v in (x, y, z, w, t.x, t.y, t.z)) or abs(x*x + y*y + z*z + w*w - 1.) > 1e-5:
        raise ValueError("invalid TF quaternion or translation")
    matrix = np.eye(4)
    matrix[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    matrix[:3, 3] = (t.x, t.y, t.z)
    return matrix


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("depth-topic", "camera-info-topic", "optical-frame", "map-frame", "output-topic"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument("--map-convention", choices=["enu"], required=True,
                        help="explicit assertion that upstream map TF is ENU")
    parser.add_argument("--depth-units", choices=["metres", "millimetres"], required=True)
    parser.add_argument("--shape", nargs=3, type=int, required=True)
    parser.add_argument("--origin", nargs=3, type=float, required=True)
    parser.add_argument("--resolution", type=float, required=True)
    parser.add_argument("--height-envelope", nargs=2, type=float, required=True,
                        help="map-frame min/max Z for full vehicle envelope; max exclusive")
    parser.add_argument("--depth-range", nargs=2, type=float, required=True)
    parser.add_argument("--max-source-age", type=float, default=.5)
    parser.add_argument("--max-info-skew", type=float, default=.1)
    parser.add_argument("--max-tf-skew", type=float, default=.02)
    parser.add_argument("--max-processing-wall", type=float, default=.5)
    parser.add_argument("--stride", type=int, default=8)
    parser.add_argument("--qos", choices=["reliable", "best_effort"], default="reliable")
    parser.add_argument("--use-sim-time", action="store_true")
    args = parser.parse_args()
    if args.map_frame == args.optical_frame:
        parser.error("ENU map frame and camera optical frame must be distinct")
    if any(not isfinite(v) or v <= 0 for v in (args.max_source_age, args.max_info_skew,
            args.max_tf_skew, args.max_processing_wall)) or args.stride < 1:
        parser.error("positive finite timing bounds and stride required")
    if not 0 < args.depth_range[0] < args.depth_range[1] or not all(isfinite(v) for v in args.depth_range):
        parser.error("positive finite ordered depth range required")
    prototype = DepthMap(tuple(args.shape), args.resolution, args.origin)
    prototype.occupancy_slice(*args.height_envelope)  # Validate before connecting.
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import Buffer, TransformListener, TransformException

    class Mapper(Node):
        def __init__(self):
            super().__init__("dronehacks_depth_mapper")
            self.set_parameters([Parameter("use_sim_time", value=args.use_sim_time)])
            qos = QoSProfile(depth=1, reliability=(ReliabilityPolicy.RELIABLE if args.qos == "reliable"
                                                 else ReliabilityPolicy.BEST_EFFORT))
            self.publisher = self.create_publisher(OccupancyGrid, args.output_topic,
                QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
            self.info = None
            self.last_source = 0
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.create_subscription(CameraInfo, args.camera_info_topic, self.on_info, qos)
            self.create_subscription(Image, args.depth_topic, self.on_image, qos)

        def on_info(self, message):
            self.info = message

        def on_image(self, image):
            begin = time.monotonic()
            try:
                source = stamp_ns(image.header.stamp)
                require_fresh(source, self.get_clock().now().nanoseconds, args.max_source_age, "depth")
                if source <= self.last_source:
                    raise ValueError("depth source did not advance; restart mapper after clock/pose reset")
                self.last_source = source
                if self.info is None:
                    raise ValueError("waiting for matching CameraInfo")
                camera = checked_camera(self.info, image, args.optical_frame, args.max_info_skew)
                tf = self.buffer.lookup_transform(args.map_frame, args.optical_frame, Time.from_msg(image.header.stamp))
                if tf.header.frame_id != args.map_frame or tf.child_frame_id != args.optical_frame:
                    raise ValueError("TF returned unexpected frames")
                if stamp_ns(tf.header.stamp) <= 0 or abs(stamp_ns(tf.header.stamp) - source) > args.max_tf_skew * 1e9:
                    raise ValueError("missing timestamped dynamic map-to-optical TF")
                mapping = DepthMap(tuple(args.shape), args.resolution, args.origin)
                stats = mapping.integrate(decode_depth(image, args.depth_units), camera, transform_matrix(tf.transform),
                    min_depth=args.depth_range[0], max_depth=args.depth_range[1], stride=args.stride)
                if stats["valid_returns"] == 0 or not np.any(mapping.cells >= 0):
                    raise ValueError("no valid observed voxels in configured map bounds")
                grid = mapping.occupancy_slice(*args.height_envelope)
                require_fresh(source, self.get_clock().now().nanoseconds, args.max_source_age, "processed depth")
                if time.monotonic() - begin > args.max_processing_wall:
                    raise ValueError("processing exceeded wall-time bound; output suppressed")
                output = OccupancyGrid()
                output.header.stamp = image.header.stamp
                output.header.frame_id = args.map_frame
                output.info.map_load_time = image.header.stamp
                output.info.resolution = grid.resolution
                output.info.width, output.info.height = grid.width, grid.height
                output.info.origin.position.x, output.info.origin.position.y = grid.origin_x, grid.origin_y
                output.info.origin.orientation.w = 1.
                output.data = list(grid.cells)
                self.publisher.publish(output)
            except (ValueError, TransformException) as error:
                self.get_logger().warning(str(error), throttle_duration_sec=2.)

    rclpy.init()
    node = Mapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
