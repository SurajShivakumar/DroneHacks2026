"""cuVSLAM stereo visual odometry as a drop-in for the OpenVINS ROS outputs.

Matches the real Sky Beta stack's estimator choice (organizer container runs
isaac_ros_visual_slam = cuVSLAM, stereo-only, IMU fusion off). This node uses
the PyCuVSLAM library (same tracker family) and publishes exactly the topics
our pipeline already consumes, so the PX4 bridge (tools/ev_bridge.py), the
supervisor's feature watchdog and the recorder work unchanged:

  /vio_shadow/odomimu      nav_msgs/Odometry  frame 'global' (gravity-up, arbitrary
                           yaw = IMU pose at start), child 'imu'; twist in IMU FLU;
                           pose covariance [position_world, rotation_IMU_local]
  /vio_shadow/points_slam  sensor_msgs/PointCloud2  one point per tracked observation
                           (count only; the watchdog counts finite points)
  /vio_shadow/points_msckf same, for readiness probes
  /vio_shadow/trackhist    sensor_msgs/Image  left image with tracked points (video)

Rig frame = IMU frame (Kalibr T_imu_cam), so world_from_rig is the IMU pose.

Consistency gate (cuVSLAM can report valid tracking while inventing rotation at
near, low-texture walls; artifacts/cuvslam-bench-v1): compare the vision angular
rate with the IMU gyro over a short window. If they disagree for longer than
`--gate-hold-s`, stop publishing pose and features. Downstream freshness checks
then fault and the supervisor lands -- PX4 is never fed an inconsistent pose.
The gate does not re-enable within a run (no estimator re-anchoring here).
"""
import argparse
from collections import deque
import json
from pathlib import Path
import time

import numpy as np
import yaml


def quat_xyzw_from_R(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t+1.)*2
        w, x, y, z = .25*s, (R[2, 1]-R[1, 2])/s, (R[0, 2]-R[2, 0])/s, (R[1, 0]-R[0, 1])/s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.+R[0, 0]-R[1, 1]-R[2, 2])*2
        w, x, y, z = (R[2, 1]-R[1, 2])/s, .25*s, (R[0, 1]+R[1, 0])/s, (R[0, 2]+R[2, 0])/s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.+R[1, 1]-R[0, 0]-R[2, 2])*2
        w, x, y, z = (R[0, 2]-R[2, 0])/s, (R[0, 1]+R[1, 0])/s, .25*s, (R[1, 2]+R[2, 1])/s
    else:
        s = np.sqrt(1.+R[2, 2]-R[0, 0]-R[1, 1])*2
        w, x, y, z = (R[1, 0]-R[0, 1])/s, (R[0, 2]+R[2, 0])/s, (R[1, 2]+R[2, 1])/s, .25*s
    q = np.array([x, y, z, w])
    return q/np.linalg.norm(q)


def R_from_quat_xyzw(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rotvec_from_R(R):
    c = np.clip((np.trace(R)-1)/2, -1, 1)
    th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    return th/(2*np.sin(th))*np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])


class GyroGate:
    """Vision vs gyro angular-rate magnitude consistency (no extrinsics needed)."""
    def __init__(self, tolerance=.35, window_s=.5, hold_s=.4):
        self.tol, self.window, self.hold = tolerance, window_s, hold_s
        self.gyro = deque()
        self.bad_since = None
        self.tripped = False
        self.last_error = 0.

    def add_gyro(self, t, w):
        self.gyro.append((t, float(np.linalg.norm(w))))
        while self.gyro and self.gyro[0][0] < t-2*self.window:
            self.gyro.popleft()

    def check(self, t, vision_rate):
        g = [v for (s, v) in self.gyro if t-self.window <= s <= t]
        if len(g) < 5:
            return True
        self.last_error = abs(vision_rate-float(np.mean(g)))
        if self.last_error > self.tol:
            self.bad_since = t if self.bad_since is None else self.bad_since
            if t-self.bad_since >= self.hold:
                self.tripped = True
        else:
            self.bad_since = None
        return not self.tripped


def build_tracker(calib_path, multicam_mode='precision', imu_calib_path=None, imu_noise_scale=1.):
    import cuvslam
    calib = yaml.safe_load(Path(calib_path).read_text().replace('%YAML:1.0', ''))
    cams = []
    for key in ('cam0', 'cam1'):
        c = calib[key]
        T = np.array(c['T_imu_cam'], float)
        fx, fy, cx, cy = c['intrinsics']
        if any(abs(v) > 1e-12 for v in c['distortion_coeffs']):
            raise ValueError('non-zero distortion not handled; extend the Distortion model')
        cams.append(cuvslam.Camera(size=c['resolution'], principal=[cx, cy], focal=[fx, fy],
                                   rig_from_camera=cuvslam.Pose(rotation=quat_xyzw_from_R(T[:3, :3]).tolist(),
                                                                translation=T[:3, 3].tolist()),
                                   distortion=cuvslam.Distortion(cuvslam.Distortion.Model.Pinhole)))
    imus = []
    if imu_calib_path is not None:
        kv = {}
        for line in Path(imu_calib_path).read_text().splitlines():
            if ':' in line:
                k, v = line.strip().split(':', 1); kv[k] = v.strip()
        # The rig frame is the IMU frame (rig_from_camera = T_imu_cam above), so rig_from_imu is identity.
        # imu_noise_scale inflates the (near-ideal) simulated IMU noise model: diagnostic for Inertial-mode divergence.
        imus = [cuvslam.ImuCalibration(rig_from_imu=cuvslam.Pose(rotation=[0, 0, 0, 1], translation=[0, 0, 0]),
                                       gyroscope_noise_density=float(kv['gyroscope_noise_density'])*imu_noise_scale,
                                       gyroscope_random_walk=float(kv['gyroscope_random_walk'])*imu_noise_scale,
                                       accelerometer_noise_density=float(kv['accelerometer_noise_density'])*imu_noise_scale,
                                       accelerometer_random_walk=float(kv['accelerometer_random_walk'])*imu_noise_scale,
                                       frequency=float(kv['update_rate']))]
    O = cuvslam.core.Odometry
    mode = {'precision': O.MulticameraMode.Precision, 'moderate': O.MulticameraMode.Moderate,
            'performance': O.MulticameraMode.Performance}[multicam_mode]
    cfg = O.Config(multicam_mode=mode, odometry_mode=O.OdometryMode.Inertial if imus else O.OdometryMode.Multicamera, use_gpu=True,
                   async_sba=True, use_motion_model=True, use_denoising=False, rectified_stereo_camera=True,
                   enable_observations_export=True, enable_landmarks_export=False)
    cuvslam.warm_up_gpu()
    return O(cuvslam.Rig(cams, imus), cfg), calib


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--calib', type=Path, required=True, help='Kalibr imucam chain (T_imu_cam, intrinsics)')
    p.add_argument('--log', type=Path, required=True, help='JSONL per-frame log')
    p.add_argument('--gate-tolerance', type=float, default=.35)
    p.add_argument('--gate-hold-s', type=float, default=.4)
    p.add_argument('--no-gate', action='store_true', help='offline diagnostics only')
    p.add_argument('--pos-std', type=float, default=.01)
    p.add_argument('--rot-std', type=float, default=.005)
    p.add_argument('--clahe', action='store_true')
    p.add_argument('--inertial', type=Path, default=None, help='Kalibr IMU yaml: enable cuVSLAM Inertial mode with /imu/data')
    p.add_argument('--swap-stereo', action='store_true', help='diagnostic: feed right as cam0, left as cam1')
    p.add_argument('--imu-noise-scale', type=float, default=1., help='diagnostic: scale the Kalibr IMU noise densities (Inertial mode)')
    a = p.parse_args()

    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, Imu, PointCloud2, PointField

    tracker, calib = build_tracker(a.calib, imu_calib_path=a.inertial, imu_noise_scale=a.imu_noise_scale)
    import cuvslam
    clahe = None
    if a.clahe:
        import cv2
        clahe = cv2.createCLAHE(clipLimit=2., tileGridSize=(8, 8))
    rclpy.init()
    node = rclpy.create_node('cuvslam_vo')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    pub_odom = node.create_publisher(Odometry, '/vio_shadow/odomimu', qos_profile_sensor_data)
    pub_slam = node.create_publisher(PointCloud2, '/vio_shadow/points_slam', qos_profile_sensor_data)
    pub_msckf = node.create_publisher(PointCloud2, '/vio_shadow/points_msckf', qos_profile_sensor_data)
    pub_hist = node.create_publisher(Image, '/vio_shadow/trackhist', qos_profile_sensor_data)
    gate = GyroGate(a.gate_tolerance, .5, a.gate_hold_s)
    left, right = {}, {}
    state = {'prev': None, 'frames': 0, 'published': 0, 'lost': 0}
    log = a.log.open('w')

    def as_mono(msg):
        buf = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        if msg.encoding in ('mono8', '8UC1'):
            img = buf[:, :msg.width]
        elif msg.encoding in ('rgb8', 'bgr8'):
            img = buf[:, :3*msg.width].reshape(msg.height, msg.width, 3).mean(axis=2).astype(np.uint8)
        else:
            raise ValueError('unsupported encoding '+msg.encoding)
        img = np.ascontiguousarray(img)
        return clahe.apply(img) if clahe is not None else img

    def cloud(stamp, n):
        m = PointCloud2()
        m.header.stamp, m.header.frame_id = stamp, 'global'
        m.height, m.width = 1, n
        m.fields = [PointField(name=c, offset=4*i, datatype=PointField.FLOAT32, count=1) for i, c in enumerate('xyz')]
        m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 12, 12*n, True
        m.data = bytes(12*n)
        return m

    def on_imu(msg):
        t = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        gate.add_gyro(t, [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        if a.inertial is not None:
            ns = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if ns > state.get('last_imu_ns', -1):
                state['last_imu_ns'] = ns
                imu_queue.append((ns, [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                                  [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]))
                if pending:
                    flush_pending()

    def process(stamp_ns, lmsg, rmsg):
        t = stamp_ns*1e-9
        t0 = time.perf_counter()
        img_l = as_mono(lmsg)
        img_r = as_mono(rmsg)
        est = tracker.track(stamp_ns, [img_r, img_l] if a.swap_stereo else [img_l, img_r])
        dt_ms = (time.perf_counter()-t0)*1e3
        obs = tracker.get_last_observations(0)
        state['frames'] += 1
        rec = {'t': t, 'track_ms': dt_ms, 'obs': len(obs)}
        if est.world_from_rig is None:
            state['lost'] += 1
            rec['valid'] = False
            log.write(json.dumps(rec)+'\n')
            return
        pz = est.world_from_rig.pose
        q = np.array([float(v) for v in pz.rotation])
        R = R_from_quat_xyzw(q)
        pos = np.array([float(v) for v in pz.translation])
        lin_local = np.zeros(3)
        ang_local = np.zeros(3)
        prev = state['prev']
        if prev is not None and t > prev[0]:
            dt = t-prev[0]
            lin_local = R.T@((pos-prev[1])/dt)
            ang_local = rotvec_from_R(prev[2].T@R)/dt
        state['prev'] = (t, pos, R)
        gate_ok = gate.check(t, float(np.linalg.norm(ang_local)))
        ok = True if a.no_gate else gate_ok
        rec.update(valid=True, p=pos.tolist(), q=q.tolist(), w=float(np.linalg.norm(ang_local)),
                   gate_err=gate.last_error, gate_ok=ok)
        log.write(json.dumps(rec)+'\n')
        if not ok:
            return  # withhold pose and features: downstream freshness faults, supervisor lands
        m = Odometry()
        m.header.stamp, m.header.frame_id, m.child_frame_id = lmsg.header.stamp, 'global', 'imu'
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = pos.tolist()
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
         m.pose.pose.orientation.z, m.pose.pose.orientation.w) = q.tolist()
        m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = lin_local.tolist()
        m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z = ang_local.tolist()
        pc = np.zeros((6, 6)); pc[:3, :3] = np.eye(3)*a.pos_std**2; pc[3:, 3:] = np.eye(3)*a.rot_std**2
        tc = np.zeros((6, 6)); tc[:3, :3] = np.eye(3)*.05**2; tc[3:, 3:] = np.eye(3)*.05**2
        m.pose.covariance, m.twist.covariance = pc.ravel().tolist(), tc.ravel().tolist()
        pub_odom.publish(m)
        c = cloud(lmsg.header.stamp, len(obs))
        pub_slam.publish(c); pub_msckf.publish(c)
        hist = img_l.copy()
        for o in obs:
            u, v = int(o.u), int(o.v)
            if 2 <= u < hist.shape[1]-2 and 2 <= v < hist.shape[0]-2:
                hist[v-2:v+3, u-2] = hist[v-2:v+3, u+2] = hist[v-2, u-2:u+3] = hist[v+2, u-2:u+3] = 255
        im = Image()
        im.header = lmsg.header
        im.height, im.width, im.encoding, im.step = hist.shape[0], hist.shape[1], 'mono8', hist.shape[1]
        im.data = hist.tobytes()
        pub_hist.publish(im)
        state['published'] += 1

    pending = {}     # Inertial mode: complete stereo pairs waiting for IMU coverage
    imu_queue = []   # Inertial mode: IMU samples not yet handed to the tracker

    def flush_pending():
        # cuVSLAM Inertial mode wants, per frame, the IMU samples up to that frame's
        # timestamp registered first and nothing newer (as in the PyCuVSLAM EuRoC
        # example); IMU arrives ahead of the images, so samples are queued and
        # released per frame. Registering newer IMU before a frame raised
        # "Timestamps are non-monotonic" (replays imu2-imu4).
        for k in sorted(pending):
            if k > state.get('last_imu_ns', -1):
                break
            lm, rm = pending.pop(k)
            if k <= state.get('last_frame_ns', -1):
                continue
            imu_queue.sort()
            while imu_queue and imu_queue[0][0] <= k:
                ns, acc, gyr = imu_queue.pop(0)
                tracker.register_imu_measurement(0, cuvslam.ImuMeasurement(
                    timestamp_ns=ns, linear_accelerations=acc, angular_velocities=gyr))
            state['last_frame_ns'] = k
            process(k, lm, rm)
        while len(pending) > 10:
            pending.pop(min(pending))

    def on_image(side):
        def cb(msg):
            k = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            (left if side == 'l' else right)[k] = msg
            other = right if side == 'l' else left
            if k in other:
                lm, rm = (msg, other.pop(k)) if side == 'l' else (other.pop(k), msg)
                (left if side == 'l' else right).pop(k, None)
                if a.inertial is not None:
                    pending[k] = (lm, rm)
                    flush_pending()
                elif k > state.get('last_frame_ns', -1):
                    state['last_frame_ns'] = k
                    process(k, lm, rm)
            for d in (left, right):
                while len(d) > 10:
                    d.pop(min(d))
        return cb

    node.create_subscription(Image, '/stereo/left/image', on_image('l'), qos_profile_sensor_data)
    node.create_subscription(Image, '/stereo/right/image', on_image('r'), qos_profile_sensor_data)
    node.create_subscription(Imu, '/imu/data', on_imu, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        traceback.print_exc()
        state['error'] = traceback.format_exc().strip().splitlines()[-1]
    finally:
        log.close()
        a.log.with_suffix('.summary.json').write_text(json.dumps({**state, 'prev': None, 'gate_tripped': gate.tripped}))
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
