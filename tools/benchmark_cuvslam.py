#!/usr/bin/env python3
"""Offline benchmark: NVIDIA cuVSLAM stereo VO vs recorded OpenVINS, vs sim truth.

OFFLINE ONLY. Reads recorded PNG stereo pairs + manifests, runs cuVSLAM
(PyCuVSLAM wheel) over a time window, and evaluates both estimators against
simulator truth poses *after the fact*. Truth is never fed to an estimator.

Usage (inside the WSL venv that has `cuvslam` installed):
  python tools/benchmark_cuvslam.py run  --bench artifacts/cuvslam-bench-v1 --tag nav
  python tools/benchmark_cuvslam.py eval --bench artifacts/cuvslam-bench-v1

`run` needs cuvslam + numpy + pillow; `eval` needs only numpy.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------- geometry

def quat_wxyz_to_R(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat_xyzw(R):
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return [x, y, z, w]


def T_from(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def umeyama_se3(src, dst):
    """Rigid (no scale) R,t minimising |dst - (R src + t)|."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T
    return R, mu_d - R @ mu_s

# ---------------------------------------------------------------- data

def load_calib(path: Path):
    """Minimal parser for the Kalibr imucam yaml written by our pipeline."""
    cams, cur = {}, None
    for line in path.read_text().splitlines():
        s = line.strip()
        if not line.startswith(" ") and s.endswith(":") and s.startswith("cam"):
            cur = s[:-1]
            cams[cur] = {}
        elif cur and ":" in s:
            k, v = s.split(":", 1)
            v = v.strip()
            if v.startswith("["):
                cams[cur][k] = json.loads(v)
            else:
                cams[cur][k] = v.strip('"')
    return cams


def load_body_from_imu(path: Path):
    if path.exists():
        return np.array(json.loads(path.read_text())["body_from_imu"], dtype=float)
    return np.eye(4)


def load_truth(path: Path):
    a = np.genfromtxt(path, delimiter=",", names=True)
    t = a["t"]
    p = np.stack([a["x"], a["y"], a["z"]], 1)
    q = np.stack([a["qw"], a["qx"], a["qy"], a["qz"]], 1)
    return t, p, q


def truth_at(truth, ts):
    t, p, q = truth
    P = np.stack([np.interp(ts, t, p[:, i]) for i in range(3)], 1)
    idx = np.clip(np.searchsorted(t, ts), 1, len(t) - 1)
    idx = np.where(np.abs(t[idx - 1] - ts) < np.abs(t[idx] - ts), idx - 1, idx)
    Rs = [quat_wxyz_to_R(q[i]) for i in idx]
    return P, Rs


def load_openvins(path: Path, body_from_imu):
    ts, Ts = [], []
    imu_from_body = np.linalg.inv(body_from_imu)
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        m = json.loads(line)
        o, p = m["orientation"], m["position"]
        R = quat_wxyz_to_R([o["w"], o["x"], o["y"], o["z"]])
        T = T_from(R, [p["x"], p["y"], p["z"]]) @ imu_from_body  # world_from_body
        ts.append(m["source_s"])
        Ts.append(T)
    return np.array(ts), Ts


def mission_window(path: Path):
    ev = {e["kind"]: e for e in json.loads(path.read_text())["events"] if "sim_s" in e}
    return ev["navigation_started"]["sim_s"], ev["request_land"]["sim_s"], ev.get("armed_observed", {}).get("sim_s")

def load_imu_noise(path: Path):
    kv = {}
    for line in path.read_text().splitlines():
        if ":" in line:
            k, v = line.strip().split(":", 1)
            kv[k] = v.strip()
    return {"gyroscope_noise_density": float(kv["gyroscope_noise_density"]),
            "gyroscope_random_walk": float(kv["gyroscope_random_walk"]),
            "accelerometer_noise_density": float(kv["accelerometer_noise_density"]),
            "accelerometer_random_walk": float(kv["accelerometer_random_walk"]),
            "frequency": float(kv["update_rate"])}


def load_ulog_imu(path: Path):
    """PX4 sensor_combined (FRD body, PX4 lockstep clock == Gazebo sim time; verified via arm time)."""
    from pyulog import ULog
    c = ULog(str(path), ["sensor_combined"]).get_dataset("sensor_combined").data
    t_ns = c["timestamp"].astype(np.int64) * 1000
    flu = np.array([1.0, -1.0, -1.0])
    gyr = np.stack([c[f"gyro_rad[{i}]"] for i in range(3)], 1) * flu
    acc = np.stack([c[f"accelerometer_m_s2[{i}]"] for i in range(3)], 1) * flu
    assert (c["accelerometer_timestamp_relative"] == 0).all()
    return {"t_ns": t_ns, "gyr": gyr, "acc": acc}

# ---------------------------------------------------------------- run cuVSLAM

def stereo_pairs(seq_dir: Path, left_name: str, right_name: str):
    man = {}
    for name in (left_name, right_name):
        d = json.loads((seq_dir / name / "sequence.json").read_text())
        man[d["topic"]] = (name, {f["stamp_ns"]: f["file"] for f in d["frames"]})
    (ln, L), (rn, Rr) = man["/stereo/left/image"], man["/stereo/right/image"]
    stamps = sorted(set(L) & set(Rr))
    return [(s, seq_dir / ln / L[s], seq_dir / rn / Rr[s]) for s in stamps], len(L), len(Rr)


def cmd_run(a):
    import cuvslam
    from PIL import Image

    bench = Path(a.bench)
    run_dir = bench / "v55"
    calib = load_calib(Path(a.calib))
    body_from_imu = load_body_from_imu(run_dir / "vision-publication.json")
    t_nav, t_land, t_arm = mission_window(Path(a.mission))
    t0 = {"nav": t_nav, "arm": t_arm, "all": 0.0}[a.start]
    pairs, nl, nr = stereo_pairs(run_dir, "image-sequence-2", "image-sequence-3")
    sel = [p for p in pairs if t0 - 1e-6 <= p[0] / 1e9 <= t_land + 1e-6]
    print(f"{len(pairs)} synced pairs (left {nl}, right {nr}); {len(sel)} in [{t0:.2f},{t_land:.2f}]")

    # Rig frame = IMU frame (Kalibr T_imu_cam). Both frames use OpenCV camera axes.
    cams = []
    for key in ("cam0", "cam1"):
        c = calib[key]
        assert c["rostopic"] == ("/stereo/left/image" if key == "cam0" else "/stereo/right/image")
        assert all(abs(v) < 1e-12 for v in c["distortion_coeffs"])
        T = np.array(c["T_imu_cam"], dtype=float)
        fx, fy, cx, cy = c["intrinsics"]
        cams.append(cuvslam.Camera(
            size=c["resolution"], principal=[cx, cy], focal=[fx, fy],
            rig_from_camera=cuvslam.Pose(rotation=R_to_quat_xyzw(T[:3, :3]), translation=T[:3, 3].tolist()),
            distortion=cuvslam.Distortion(cuvslam.Distortion.Model.Pinhole)))
    imus, imu_cal = [], None
    if a.imu_ulog:
        # Rig frame = IMU frame (FLU), so rig_from_imu = identity. Noise from the run's Kalibr IMU yaml.
        imu_cal = load_imu_noise(Path(a.imu_calib))
        imus = [cuvslam.ImuCalibration(rig_from_imu=cuvslam.Pose(rotation=[0, 0, 0, 1], translation=[0, 0, 0]),
                                       **imu_cal)]
        imu = load_ulog_imu(Path(a.imu_ulog))
        it = np.searchsorted(imu["t_ns"], int(sel[0][0]) - int(0.5e9))  # IMU from 0.5 s before first frame
    rig = cuvslam.Rig(cams, imus)
    O = cuvslam.core.Odometry
    mode = {"precision": O.MulticameraMode.Precision, "moderate": O.MulticameraMode.Moderate,
            "performance": O.MulticameraMode.Performance}[a.multicam_mode]
    cfg = O.Config(multicam_mode=mode, odometry_mode=O.OdometryMode.Inertial if a.imu_ulog else O.OdometryMode.Multicamera, use_gpu=True,
                   async_sba=not a.sync_sba, use_motion_model=not a.no_motion_model,
                   use_denoising=a.denoise, rectified_stereo_camera=not a.unrectified,
                   enable_observations_export=True, enable_landmarks_export=False,
                   max_frame_delta_s=a.max_frame_delta_s)
    if a.preprocess == "clahe":  # same histogram method OpenVINS uses (estimator_config histogram_method CLAHE)
        import cv2
        clahe = cv2.createCLAHE(clipLimit=a.clahe_clip, tileGridSize=(8, 8))
        prep = lambda im: np.ascontiguousarray(clahe.apply(np.ascontiguousarray(im)))
    else:
        prep = np.ascontiguousarray
    cuvslam.warm_up_gpu()
    odo = O(rig, cfg)

    out = bench / f"cuvslam-poses-{a.tag}.jsonl"
    imu_from_body = np.linalg.inv(body_from_imu)
    n_valid = 0
    with out.open("w") as f:
        for i, (stamp, lp, rp) in enumerate(sel):
            tl = time.perf_counter()
            imgs = [prep(np.asarray(Image.open(lp).convert("L"))), prep(np.asarray(Image.open(rp).convert("L")))]
            tt = time.perf_counter()
            if a.imu_ulog:
                while it < len(imu["t_ns"]) and imu["t_ns"][it] <= stamp:
                    odo.register_imu_measurement(0, cuvslam.ImuMeasurement(
                        timestamp_ns=int(imu["t_ns"][it]), linear_accelerations=imu["acc"][it].tolist(),
                        angular_velocities=imu["gyr"][it].tolist()))
                    it += 1
            est = odo.track(stamp, imgs)
            te = time.perf_counter()
            nobs = len(odo.get_last_observations(0))
            rec = {"t": stamp / 1e9, "frame": lp.name, "load_ms": (tt - tl) * 1e3,
                   "track_ms": (te - tt) * 1e3, "valid": est.world_from_rig is not None, "obs_left": nobs}
            if rec["valid"]:
                pz = est.world_from_rig.pose
                qx, qy, qz, qw = [float(v) for v in pz.rotation]
                T = T_from(quat_wxyz_to_R([qw, qx, qy, qz]), [float(v) for v in pz.translation])
                T = T @ imu_from_body  # world_from_body (world = rig at first frame)
                rec["T"] = T.tolist()
                n_valid += 1
            f.write(json.dumps(rec) + "\n")
    meta = {"tag": a.tag, "start": a.start, "t_start": t0, "t_end": t_land, "frames": len(sel),
            "valid": n_valid, "cuvslam_version": cuvslam.get_version()[0],
            "config": {"multicam_mode": a.multicam_mode, "odometry_mode": ("Inertial (stereo + PX4 ULog sensor_combined IMU)" if a.imu_ulog
                                                          else "Multicamera (stereo VO, no IMU)"),
                       "imu_calibration": imu_cal,
                       "use_gpu": True, "async_sba": not a.sync_sba, "use_motion_model": not a.no_motion_model,
                       "use_denoising": a.denoise, "rectified_stereo_camera": not a.unrectified,
                       "max_frame_delta_s": a.max_frame_delta_s, "preprocess": a.preprocess,
                       "clahe_clip": a.clahe_clip if a.preprocess == "clahe" else None},
            "synced_pairs_total": len(pairs), "left_frames": nl, "right_frames": nr}
    (bench / f"cuvslam-meta-{a.tag}.json").write_text(json.dumps(meta, indent=2))
    print(json.dumps(meta, indent=2))

# ---------------------------------------------------------------- evaluation

def evaluate(name, ts, Ts, truth, t_nav, t_land, segments_reset=None):
    m = (ts >= t_nav) & (ts <= t_land)
    ts = ts[m]
    Ts = [T for T, k in zip(Ts, m) if k]
    P_est = np.array([T[:3, 3] for T in Ts])
    P_gt, R_gt = truth_at(truth, ts)
    seg = np.linalg.norm(np.diff(P_gt[:, :2], axis=0), axis=1)
    path_xy = float(seg.sum())
    res = {"samples": int(len(ts)), "t_first": float(ts[0]), "t_last": float(ts[-1]),
           "truth_path_xy_m": path_xy}

    def stats(Pa, tag, Ra=None):
        e = Pa - P_gt
        exy = np.linalg.norm(e[:, :2], axis=1)
        e3 = np.linalg.norm(e, axis=1)
        d = {"xy_rms_m": float(np.sqrt((exy ** 2).mean())), "xy_max_m": float(exy.max()),
             "xy_final_m": float(exy[-1]), "z_rms_m": float(np.sqrt((e[:, 2] ** 2).mean())),
             "xyz_rms_m": float(np.sqrt((e3 ** 2).mean())),
             "xy_final_per_m_travelled_pct": 100 * float(exy[-1]) / path_xy,
             "xy_max_per_m_travelled_pct": 100 * float(exy.max()) / path_xy,
             "t_of_xy_max": float(ts[int(exy.argmax())])}
        if Ra is not None:
            yerr = np.array([abs(wrap(yaw_of(Re) - yaw_of(Rg))) for Re, Rg in zip(Ra, R_gt)])
            d["yaw_err_deg_rms"] = float(np.degrees(np.sqrt((yerr ** 2).mean())))
            d["yaw_err_deg_max"] = float(np.degrees(yerr.max()))
        res[tag] = d

    # (1) SE3 Umeyama on all positions in the interval (best-case shape fit, no scale)
    R, t = umeyama_se3(P_est, P_gt)
    stats((R @ P_est.T).T + t, "align_se3_umeyama_interval", [R @ T[:3, :3] for T in Ts])
    # (2) Anchor at first sample of the interval (full pose: T_align = T_gt(t0) inv(T_est(t0)))
    T0 = T_from(R_gt[0], P_gt[0]) @ np.linalg.inv(Ts[0])
    Ta = [T0 @ T for T in Ts]
    stats(np.array([T[:3, 3] for T in Ta]), "align_first_frame_pose", [T[:3, :3] for T in Ta])
    # (3) First-frame position + yaw only (gravity-aligned world assumed) -- the "deployable" alignment
    dy = wrap(yaw_of(R_gt[0]) - yaw_of(Ts[0][:3, :3]))
    Rz = np.array([[math.cos(dy), -math.sin(dy), 0], [math.sin(dy), math.cos(dy), 0], [0, 0, 1]])
    Py = (Rz @ (P_est - P_est[0]).T).T + P_gt[0]
    stats(Py, "align_first_frame_pos_yaw", [Rz @ T[:3, :3] for T in Ts])
    # Relative pose error over ~1 s windows (alignment-free; body frame at window start)
    rte, rre = [], []
    for i in range(len(ts)):
        j = int(np.searchsorted(ts, ts[i] + 1.0))
        if j >= len(ts):
            break
        G = np.linalg.inv(T_from(R_gt[i], P_gt[i])) @ T_from(R_gt[j], P_gt[j])
        E = np.linalg.inv(Ts[i]) @ Ts[j]
        rte.append(float(np.linalg.norm(E[:3, 3] - G[:3, 3])))
        rre.append(math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(G[:3, :3].T @ E[:3, :3]) - 1) / 2)))))
    rte, rre = np.array(rte), np.array(rre)
    res["rpe_1s"] = {"windows": int(len(rte)), "trans_median_m": float(np.median(rte)),
                     "trans_p90_m": float(np.percentile(rte, 90)), "trans_max_m": float(rte.max()),
                     "rot_median_deg": float(np.median(rre)), "rot_p90_deg": float(np.percentile(rre, 90)),
                     "frac_windows_trans_gt_5cm": float((rte > 0.05).mean())}
    # step jumps (estimate vs truth increments), a proxy for glitches
    dE = np.diff(P_est, axis=0)
    dG = np.diff(P_gt, axis=0)
    inc = np.linalg.norm((np.array([R @ v for v in dE]) - dG)[:, :2], axis=1)
    res["step_xy_error_max_m"] = float(inc.max()) if len(inc) else 0.0
    res["step_xy_error_gt_5cm"] = int((inc > 0.05).sum())
    return res


def cmd_eval(a):
    bench = Path(a.bench)
    run_dir = bench / "v55"
    truth = load_truth(Path(a.truth))
    t_nav, t_land, _ = mission_window(Path(a.mission))
    body_from_imu = load_body_from_imu(run_dir / "vision-publication.json")
    out = {"run": "gb10-sky-beta-course-v55", "interval": {"navigation_started_s": t_nav, "request_land_s": t_land},
           "notes": [
               "Truth (simulator base_link pose, course frame) used only after the fact for evaluation.",
               "Estimates converted IMU->body with vision-publication.json body_from_imu before comparison.",
               "XY errors computed after each alignment; drift % = XY error / truth XY path length over interval.",
           ], "estimators": {}}

    # OpenVINS (live, stereo+IMU, 15 Hz tracking; recorded odom is a subsampled log)
    ts, Ts = load_openvins(run_dir / "vio-odometry.jsonl", body_from_imu)
    allts = ts
    r = evaluate("openvins", ts, Ts, truth, t_nav, t_land)
    inn = allts[(allts >= t_nav) & (allts <= t_land)]
    gaps = np.diff(inn)
    r["log_rate_hz"] = float(len(inn) / (inn[-1] - inn[0]))
    r["log_gap_max_s"] = float(gaps.max())
    r["log_gaps_gt_1s"] = int((gaps > 1.0).sum())
    r["tracking_lost_events"] = r["log_gaps_gt_1s"]
    r["source"] = "vio-odometry.jsonl (/vio_shadow/odomimu, live OpenVINS stereo+IMU)"
    out["estimators"]["openvins"] = r

    for meta_p in sorted(bench.glob("cuvslam-meta-*.json")):
        meta = json.loads(meta_p.read_text())
        recs = [json.loads(l) for l in (bench / f"cuvslam-poses-{meta['tag']}.jsonl").read_text().splitlines()]
        valid = [x["valid"] for x in recs]
        lost_events = sum(1 for i in range(1, len(valid)) if valid[i - 1] and not valid[i])
        first_valid = next(i for i, v in enumerate(valid) if v)
        # cuVSLAM keeps one world frame until tracking is lost; evaluate the first continuous segment
        # if there was a loss (later segments would need their own re-anchoring).
        seg_end = next((i for i in range(first_valid, len(valid)) if not valid[i]), len(valid))
        segrecs = recs[first_valid:seg_end]
        ts = np.array([x["t"] for x in segrecs])
        Ts = [np.array(x["T"]) for x in segrecs]
        r = evaluate("cuvslam", ts, Ts, truth, t_nav, t_land)
        tm = np.array([x["track_ms"] for x in recs[1:]])
        lm = np.array([x["load_ms"] for x in recs[1:]])
        r.update({
            "frames_fed": len(recs), "frames_valid": int(sum(valid)), "frames_invalid": int(len(valid) - sum(valid)),
            "tracking_lost_events": lost_events, "first_continuous_segment_frames": len(segrecs),
            "first_continuous_segment_covers_interval_to_s": float(recs[seg_end - 1]["t"]),
            "obs_left_median": float(np.median([x["obs_left"] for x in recs])),
            "obs_left_min": int(min(x["obs_left"] for x in recs)),
            "input_rate_hz": float(len(recs) / (recs[-1]["t"] - recs[0]["t"])),
            "latency_track_ms": {"median": float(np.median(tm)), "p95": float(np.percentile(tm, 95)),
                                 "max": float(tm.max()), "first_frame": float(recs[0]["track_ms"])},
            "png_decode_ms_median": float(np.median(lm)),
            "meta": meta})
        out["estimators"][f"cuvslam_{meta['tag']}"] = r

    (bench / "result.json").write_text(json.dumps(out, indent=2))
    # compact table
    print(f"{'estimator':22s} {'align':28s} {'xyRMS':>7s} {'xyMax':>7s} {'xyEnd':>7s} {'end%':>6s} {'yawRMS':>7s}")
    for k, r in out["estimators"].items():
        for al in ("align_se3_umeyama_interval", "align_first_frame_pose", "align_first_frame_pos_yaw"):
            s = r[al]
            print(f"{k:22s} {al:28s} {s['xy_rms_m']:7.3f} {s['xy_max_m']:7.3f} {s['xy_final_m']:7.3f} "
                  f"{s['xy_final_per_m_travelled_pct']:6.2f} {s.get('yaw_err_deg_rms', float('nan')):7.2f}")
        q = r["rpe_1s"]
        print(f"   rpe1s trans med/p90 {q['trans_median_m']:.3f}/{q['trans_p90_m']:.3f} m rot med/p90 "
              f"{q['rot_median_deg']:.2f}/{q['rot_p90_deg']:.2f} deg  >5cm {100*q['frac_windows_trans_gt_5cm']:.0f}%")
        print(f"   samples={r['samples']} lost={r['tracking_lost_events']} path={r['truth_path_xy_m']:.2f} m "
              f"stepmax={r['step_xy_error_max_m']:.3f}" + (f" lat={r['latency_track_ms']}" if 'latency_track_ms' in r else ""))


def main():
    root = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["run", "eval"])
    ap.add_argument("--bench", default=str(root / "artifacts/cuvslam-bench-v1"))
    ap.add_argument("--calib", default=str(root / "artifacts/vio-feature-budget-v1/kalibr_imucam_chain.yaml"))
    ap.add_argument("--mission", default=str(root / "artifacts/gb10-sky-beta-course-v55/course-mission.json"))
    ap.add_argument("--truth", default=str(root / "artifacts/gb10-sky-beta-course-v55/truth-poses.csv"))
    ap.add_argument("--tag", default="nav")
    ap.add_argument("--start", choices=["nav", "arm", "all"], default="nav",
                    help="where cuVSLAM starts tracking; evaluation is always the navigation interval")
    ap.add_argument("--multicam-mode", choices=["precision", "moderate", "performance"], default="precision")
    ap.add_argument("--sync-sba", action="store_true")
    ap.add_argument("--max-frame-delta-s", type=float, default=1.0)
    ap.add_argument("--no-motion-model", action="store_true")
    ap.add_argument("--denoise", action="store_true")
    ap.add_argument("--unrectified", action="store_true", help="do not tell cuVSLAM the pair is rectified")
    ap.add_argument("--preprocess", choices=["none", "clahe"], default="none")
    ap.add_argument("--clahe-clip", type=float, default=2.0)
    ap.add_argument("--imu-ulog", default="", help="PX4 ULog; uses sensor_combined (FRD->FLU) and Inertial mode")
    ap.add_argument("--imu-calib", default=str(root / "artifacts/cuvslam-bench-v1/v55/settings/kalibr_imu_chain.yaml"))
    a = ap.parse_args()
    cmd_run(a) if a.cmd == "run" else cmd_eval(a)


if __name__ == "__main__":
    main()
