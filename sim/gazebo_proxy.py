"""Assemble a no-GNSS Gazebo proxy from pinned upstream X500 dynamics.

This is simulator setup, not a flight algorithm. Keeps upstream motor dynamics
and removes NavSat from a COPY. It neither publishes odometry nor arms PX4.
Camera poses/FOV and ranging are illustrative, not Sky Beta calibration.
"""

import argparse
from copy import deepcopy
from hashlib import sha256
import json
from math import isfinite
from pathlib import Path
import xml.etree.ElementTree as ET


def value(parent, name, text, **attributes):
    element = ET.SubElement(parent, name, **attributes)
    element.text = str(text)
    return element


def camera(link, name, y, kind="camera"):
    sensor = ET.SubElement(link, "sensor", name=name, type=kind)
    # Match the pinned upstream x500_depth camera's forward/height offset.
    value(sensor, "pose", f".13233 {y} .26078 0 0 0")
    value(sensor, "always_on", "true")
    value(sensor, "update_rate", 15)
    value(sensor, "topic", f"/dronehacks/{name}")
    cam = ET.SubElement(sensor, "camera")
    value(cam, "horizontal_fov", "1.3962634")
    value(cam, "camera_info_topic", f"/dronehacks/{name}/camera_info")
    image = ET.SubElement(cam, "image")
    value(image, "width", 640)
    value(image, "height", 400)
    value(image, "format", "L8" if name in ("left", "right") else "R8G8B8")
    clip = ET.SubElement(cam, "clip")
    value(clip, "near", .1)
    value(clip, "far", 10)


def build(models, course, output, start_pose=(-2.5146, .9144, .3, 1.57079632679),
          hide_airframe_visuals=False):
    models, course, output = Path(models), Path(course), Path(output)
    if output.exists():
        raise FileExistsError(f"preserve the old scene; choose new output: {output}")
    base_path, motor_path = models / "x500_base/model.sdf", models / "x500/model.sdf"
    base = ET.parse(base_path).find("model")
    motor_model = ET.parse(motor_path).find("model")
    proxy = deepcopy(base)
    proxy.set("name", "dronehacks_x500_proxy")
    removed = []
    for link in proxy.findall("link"):
        if hide_airframe_visuals:
            for visual in list(link.findall("visual")):
                link.remove(visual)
        for sensor in list(link.findall("sensor")):
            if sensor.get("type") == "navsat":
                removed.append(sensor.get("name"))
                link.remove(sensor)
    if not removed:
        raise ValueError("expected upstream NavSat sensor was not found; inspect source version")
    for plugin in motor_model.findall("plugin"):
        proxy.append(deepcopy(plugin))
    body = proxy.find("link[@name='base_link']")
    if body is None:
        raise ValueError("upstream base_link is missing")
    camera(body, "left", .0375)
    camera(body, "right", -.0375)
    camera(body, "rgb", 0)
    camera(body, "ideal_depth", 0, "depth_camera")
    sensor = ET.SubElement(body, "sensor", name="down_range", type="gpu_lidar")
    value(sensor, "pose", "0 0 -.05 0 1.57079632679 0")
    value(sensor, "topic", "/dronehacks/down_range")
    value(sensor, "update_rate", 30)
    lidar = ET.SubElement(sensor, "lidar")
    horizontal = ET.SubElement(ET.SubElement(lidar, "scan"), "horizontal")
    for tag, text in (("samples", 1), ("resolution", 1), ("min_angle", 0), ("max_angle", 0)):
        value(horizontal, tag, text)
    distance = ET.SubElement(lidar, "range")
    for tag, text in (("min", .05), ("max", 4), ("resolution", .01)):
        value(distance, tag, text)
    root = ET.parse(course)
    world = root.find("world")
    for name in ("Imu", "AirPressure", "Magnetometer"):
        filename = {"Imu": "imu", "AirPressure": "air-pressure", "Magnetometer": "magnetometer"}[name]
        ET.SubElement(world, "plugin", filename=f"gz-sim-{filename}-system",
                      name=f"gz::sim::systems::{name}")
    if len(start_pose) != 4 or not all(isfinite(x) for x in start_pose):
        raise ValueError("start pose must be finite XYZ and yaw in radians")
    x, y, z, yaw = start_pose
    for old_pose in list(proxy.findall("pose")):
        proxy.remove(old_pose)
    value(proxy, "pose", f"{x} {y} {z} 0 0 {yaw}")
    world.append(proxy)
    output.mkdir(parents=True)
    ET.indent(root)
    root.write(output / "world.sdf", encoding="utf-8", xml_declaration=True)
    metadata = {
        "kind": "no_gnss_x500_proxy_not_sky_beta",
        "upstream_sources": {"x500_base": sha256(base_path.read_bytes()).hexdigest(),
                             "x500_motors": sha256(motor_path.read_bytes()).hexdigest()},
        "removed_navsat_sensors": removed, "position_truth_publisher": False,
        "airframe_visuals_hidden_for_diagnosis": hide_airframe_visuals,
        "camera_contract": "illustrative 640x400 at 15 Hz; stereo baseline 0.075 m",
        "camera_body_offset_xz": [.13233, .26078],
        "depth": "ideal Gazebo depth, not stereo reconstruction",
        "range": "ideal single-beam scan, not qualified VL53L1X response",
        "calibration_tf_validated": False,
        "start_pose_xyz_yaw": list(start_pose),
        "world_sha256": sha256((output / "world.sdf").read_bytes()).hexdigest(),
    }
    (output / "manifest.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return metadata


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--course", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--diagnostic-hide-airframe-visuals", action="store_true")
    parser.add_argument("--start-pose", nargs=4, type=float,
                        default=(-2.5146, .9144, .3, 1.57079632679), metavar=("X", "Y", "Z", "YAW"))
    args = parser.parse_args()
    print(json.dumps(build(args.models, args.course, args.output, args.start_pose,
                           args.diagnostic_hide_airframe_visuals), indent=2))
