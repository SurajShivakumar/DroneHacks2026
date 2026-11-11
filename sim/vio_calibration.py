"""Derive proxy VIO calibration from SDF and actual CameraInfo observations.

This calibrates virtual sensors only. No world pose, simulator odometry, or
position fix is read. Do not deploy this calibration on a real camera/aircraft.
"""

import argparse
from hashlib import sha256
import json
from math import cos, isfinite, sin, sqrt, tan
from pathlib import Path
import xml.etree.ElementTree as ET


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rotation(r, p, y):
    rx = [[1, 0, 0], [0, cos(r), -sin(r)], [0, sin(r), cos(r)]]
    ry = [[cos(p), 0, sin(p)], [0, 1, 0], [-sin(p), 0, cos(p)]]
    rz = [[cos(y), -sin(y), 0], [sin(y), cos(y), 0], [0, 0, 1]]
    return multiply(multiply(rz, ry), rx)


def optical_to_imu(camera_pose, imu_pose):
    """Kalibr T_imu_cam: optical XYZ -> IMU FLU, including lever arm."""
    body_imu = rotation(*imu_pose[3:])
    imu_body = [list(row) for row in zip(*body_imu)]
    sensor_optical = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    orientation = multiply(multiply(imu_body, rotation(*camera_pose[3:])), sensor_optical)
    delta = [camera_pose[k] - imu_pose[k] for k in range(3)]
    position = [sum(imu_body[i][k] * delta[k] for k in range(3)) for i in range(3)]
    return [orientation[i] + [position[i]] for i in range(3)] + [[0, 0, 0, 1]]


def pose_of(sensor):
    values = list(map(float, (sensor.findtext("pose") or "0 0 0 0 0 0").split()))
    if len(values) != 6 or not all(isfinite(x) for x in values):
        raise ValueError("sensor pose must be a finite six-element XYZ/RPY transform")
    if sensor.find("pose") is not None and sensor.find("pose").get("relative_to"):
        raise ValueError("explicit relative_to requires TF resolution; refusing a guessed transform")
    return values


def checked_intrinsics(camera_info, sensor):
    cam = sensor.find("camera")
    width, height = int(cam.findtext("image/width")), int(cam.findtext("image/height"))
    if [camera_info["width"], camera_info["height"]] != [width, height]:
        raise ValueError("CameraInfo dimensions do not match the sensor model")
    k = camera_info["k"]
    if len(k) != 9 or not all(isfinite(float(x)) for x in k) or min(k[0], k[4]) <= 0:
        raise ValueError("invalid intrinsic matrix")
    expected_fx = width / (2 * tan(float(cam.findtext("horizontal_fov")) / 2))
    if abs(k[0] - expected_fx) > 1e-3 or abs(k[4] - expected_fx) > 1e-3:
        raise ValueError("intrinsics do not agree with the virtual pinhole sensor")
    if any(abs(x) > 1e-12 for x in camera_info["d"]):
        raise ValueError("nonzero distortion requires a measured distortion model")
    return [k[0], k[4], k[2], k[5]]


def yaml_document(mapping):
    def emit(items, depth=0):
        lines = []
        for key, value in items.items():
            if isinstance(value, dict):
                lines.append(" " * depth + key + ":")
                lines.extend(emit(value, depth + 2))
            else:
                lines.append(" " * depth + key + ": " + json.dumps(value, allow_nan=False))
        return lines
    return "%YAML:1.0\n" + "\n".join(emit(mapping)) + "\n"


def build(world, observations, output, model_name='dronehacks_x500_proxy'):
    if output.exists():
        raise FileExistsError("choose a fresh calibration directory")
    doc = ET.parse(world)
    link = doc.find(f"world/model[@name='{model_name}']/link[@name='base_link']")
    if link is None:
        raise ValueError("expected proxy base_link not found")
    imu = link.find("sensor[@type='imu']")
    if imu is None:
        raise ValueError("IMU sensor not found")
    report = json.loads(observations.read_text())
    imu_stream = report["streams"]["/imu/data"]
    span = imu_stream["source_last_s"] - imu_stream["source_first_s"]
    if span <= 0 or imu_stream["messages"] < 100 or imu_stream["source_regressions"]:
        raise ValueError("insufficient monotonic IMU evidence")
    rate = (imu_stream["messages"] - 1) / span
    if not 200 <= rate <= 300:
        raise ValueError("observed IMU rate is outside the expected proxy range")
    chains = {}
    for i, eye in enumerate(("left", "right")):
        sensor = link.find(f"sensor[@name='{eye}']")
        fields = report["streams"][f"/stereo/{eye}/camera_info"]["latest_fields"]
        chains[f"cam{i}"] = {
            "T_imu_cam": optical_to_imu(pose_of(sensor), pose_of(imu)),
            "cam_overlaps": [1 - i], "camera_model": "pinhole", "distortion_model": "radtan",
            "distortion_coeffs": [0., 0., 0., 0.],
            "intrinsics": checked_intrinsics(fields, sensor),
            "resolution": [fields["width"], fields["height"]],
            "rostopic": f"/stereo/{eye}/image",
        }
    if any(abs(chains["cam0"]["T_imu_cam"][i][j] - chains["cam1"]["T_imu_cam"][i][j]) > 1e-9
           for i in range(3) for j in range(3)):
        raise ValueError("non-rectified stereo needs separate validation")
    left, right = (chains[f"cam{i}"]["T_imu_cam"] for i in (0, 1))
    separation = [right[k][3] - left[k][3] for k in range(3)]
    baseline = sum(left[k][0] * separation[k] for k in range(3))
    right_info = report["streams"]["/stereo/right/camera_info"]["latest_fields"]
    observed_baseline = -right_info["p"][3] / right_info["p"][0]
    if baseline <= 0 or abs(baseline - observed_baseline) > 1e-5:
        raise ValueError("right projection baseline disagrees with sensor extrinsics")
    def noise_max(kind):
        return max(float(imu.findtext(f"imu/{kind}/{axis}/noise/stddev")) for axis in "xyz")
    imu_chain = {"imu0": {
        "T_i_b": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
        "accelerometer_noise_density": noise_max("linear_acceleration") / sqrt(rate),
        "gyroscope_noise_density": noise_max("angular_velocity") / sqrt(rate),
        # The proxy has no bias process. These small nonzero allowances model
        # uncertainty; they are not measured hardware bias/calibration values.
        "accelerometer_random_walk": 1e-5, "gyroscope_random_walk": 1e-6,
        "rostopic": "/imu/data", "time_offset": 0., "update_rate": rate, "model": "kalibr",
        "Tw": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "Ta": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "R_IMUtoGYRO": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "R_IMUtoACC": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "Tg": [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
    }}
    settings = {
        "verbosity": "INFO", "use_fej": True, "integration": "rk4", "use_stereo": True,
        "max_cameras": 2, "calib_cam_extrinsics": False, "calib_cam_intrinsics": False,
        "calib_cam_timeoffset": False, "calib_imu_intrinsics": False,
        "calib_imu_g_sensitivity": False, "max_clones": 11, "max_slam": 50,
        "max_slam_in_update": 25, "max_msckf_in_update": 40, "dt_slam_delay": 1.,
        "gravity_mag": 9.80665, "feat_rep_msckf": "GLOBAL_3D",
        "feat_rep_slam": "ANCHORED_MSCKF_INVERSE_DEPTH", "feat_rep_aruco": "GLOBAL_3D",
        "try_zupt": True, "zupt_chi2_multipler": 1., "zupt_max_velocity": .1,
        "zupt_noise_multiplier": 10., "zupt_max_disparity": .5, "zupt_only_at_beginning": True,
        "init_window_time": 2., "init_imu_thresh": 1.5, "init_max_disparity": 10.,
        "init_max_features": 50, "init_dyn_use": False,
        "use_klt": True, "num_pts": 200, "fast_threshold": 20, "grid_x": 5,
        "grid_y": 5, "min_px_dist": 10, "knn_ratio": .7, "track_frequency": 15.,
        "downsample_cameras": False, "num_opencv_threads": 2, "histogram_method": "HISTOGRAM",
        "use_aruco": False, "use_mask": False, "record_timing_information": False,
        "save_total_state": False, "up_msckf_sigma_px": 1., "up_msckf_chi2_multipler": 1.,
        "up_slam_sigma_px": 1., "up_slam_chi2_multipler": 1.,
        "relative_config_imu": "kalibr_imu_chain.yaml", "relative_config_imucam": "kalibr_imucam_chain.yaml",
    }
    output.mkdir(parents=True)
    for name, contents in (("kalibr_imucam_chain.yaml", chains), ("kalibr_imu_chain.yaml", imu_chain),
                            ("estimator_config.yaml", settings)):
        (output / name).write_text(yaml_document(contents), encoding="utf-8")
    metadata = {"kind": "virtual_sensor_calibration_not_hardware", "baseline_m": baseline,
                "observed_imu_hz_sim": rate, "world_sha256": sha256(world.read_bytes()).hexdigest(),
                "observation_sha256": sha256(observations.read_bytes()).hexdigest(),
                "bias_random_walk": "small conservative allowances; no modeled bias process",
                "initialization": "standard static IMU/disparity checks with initial-only ZUPT; no truth pose",
                "position_fix_published": False}
    (output / "calibration.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return metadata


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--observations", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model-name", default='dronehacks_x500_proxy')
    args = parser.parse_args()
    print(json.dumps(build(args.world, args.observations, args.output, args.model_name), indent=2))
