"""Bounded sensor/ROS test; optional explicit approved simulation mission.

Run under sourced ROS on Linux. Every child process is stopped on exit. This is
an infrastructure test and does not establish flight readiness or estimator health.
"""

import argparse
from hashlib import sha256
import json
from math import isfinite
import os
import re
from pathlib import Path
import signal
import shlex
import socket
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


def observation_seconds(*, vio_config, inspect_hover=False, inspect_offboard_disarmed=False):
    return 195 if inspect_offboard_disarmed else (130 if inspect_hover else (90 if vio_config is not None else 30))


def course_model_for_scene(project, scene):
    includes=[node.text for node in ET.parse(scene).findall('world/include/uri')
              if node.text and node.text.startswith('model://dronehacks_course')]
    if len(includes)!=1:
        raise ValueError('one explicit course asset is required for evidence')
    name=includes[0].removeprefix('model://')
    if '/' in name or '\\' in name or name in ('.','..'):
        raise ValueError('invalid course model reference')
    candidates=[scene.parent.parent/name/'model.sdf',
                project/'artifacts/course-v3-r6/models'/name/'model.sdf']
    found=[path for path in candidates if path.is_file()]
    if len(found)!=1:
        raise ValueError('course asset missing or ambiguous')
    return found[0]


def disarmed_offboard_passed(report, exit_code):
    """Diagnostic mode/preflight evidence only; never a flight-ready assertion."""
    if not isinstance(report, dict):
        return False
    telemetry = report.get('telemetry', {})
    if not isinstance(telemetry, dict):
        return False
    status = telemetry.get('status', {}).get('last_fields', {})
    land = telemetry.get('land', {}).get('last_fields', {})
    at_least = lambda value, minimum: type(value) in (int, float) and isfinite(value) and value >= minimum
    return (type(exit_code) is int and exit_code == 0 and report.get('state') == 'COMPLETE'
            and report.get('probe_mode') is True and report.get('control_publishers_created') is True
            and report.get('startup_window_satisfied') is True
            and at_least(report.get('startup_window_s'), 30)
            and report.get('offboard_preflight_window_satisfied') is True
            and at_least(report.get('offboard_preflight_window_s'), 30)
            and report.get('mode_ack_accepted') is True and type(report.get('mode_commands_sent')) is int
            and report.get('mode_commands_sent') == 1 and at_least(report.get('heartbeat_samples_published'), 22)
            and report.get('exception') is None and status.get('offboard') is True
            and status.get('preflight') is True and status.get('armed') is False
            and status.get('disarmed') is True and land.get('landed') is True)


def mission_result_passed(report,exit_code,execute):
    if not isinstance(report,dict) or exit_code!=0 or report.get('exception') is not None:
        return False
    streams=report.get('streams',{})
    status=streams.get('status',{}).get('last_fields') or {}
    land=streams.get('land',{}).get('last_fields') or {}
    if status.get('disarmed') is not True or status.get('armed') is not False or land.get('landed') is not True:
        return False
    if not execute:
        elapsed=report.get('idle_health_window_s')
        return (report.get('state')=='WAITING' and report.get('control_publishers_created') is False
                and type(elapsed) in (int,float) and isfinite(elapsed) and elapsed>=30)
    return (report.get('state')=='COMPLETE' and report.get('land_acknowledged') is True
            and report.get('land_mode_observed') is True and report.get('control_publishers_created') is True
            and {'arm','land'}<=set(report.get('commands_published',[])))


def run(project, runtime, scene, output, with_px4=False, vio_config=None, hardware_rendering=False,
        record_image=False, diagnostic_rig=False, external_vision=False,
        px4_model_name='dronehacks_x500_proxy', bootstrap_fixture=False,
        ground_station=None, inspect_hover=False, align_heading=False, inspect_offboard_disarmed=False,
        airframe_contract=None,mission_settings=None,execute_mission=False,approval_reference=None,
        mission_repo=None,mission_policy=None,mission_trajectory=None,record_topic='/camera/rgb/image',
        record_vio_odometry=False,diagnostic_rig_amplitude=.12,
        mission_observe_at_start=False,mission_observe_seconds=40.,diagnostic_yaw_rad=0.,record_all_vio_features=False,
        record_ideal_depth=False,diagnostic_long_profile=False,record_bag=False,estimator='openvins',skybeta_profile=False,team_nav_args=(),skybeta_mag_type=None,cuvslam_inertial=None,native_program=None):
    airframe=json.loads(airframe_contract.read_text()) if airframe_contract else None
    if record_ideal_depth and execute_mission:
        raise ValueError('ideal depth geometry evidence is restricted to disarmed diagnostics')
    if diagnostic_long_profile and (execute_mission or not bootstrap_fixture or not diagnostic_rig):
        raise ValueError('long fixture profile requires disarmed bootstrap diagnostic')
    if not 0<=diagnostic_yaw_rad<=1.6 or (diagnostic_yaw_rad and
            (not bootstrap_fixture or execute_mission or inspect_offboard_disarmed or inspect_hover)):
        raise ValueError('yaw fixture is bounded to 0..1.6 rad and disarmed observation only')
    if mission_observe_at_start and (not mission_settings or execute_mission):
        raise ValueError('early mission observer requires readonly mission settings and cannot execute')
    if mission_observe_at_start and not 30<=mission_observe_seconds<=120:
        raise ValueError('readonly mission observer duration must be 30..120 wall seconds')
    if execute_mission and not (mission_settings and approval_reference and approval_reference.strip()):
        raise ValueError('mission execution requires concrete settings and recorded team approval')
    if mission_settings and (inspect_hover or inspect_offboard_disarmed or not (bootstrap_fixture and ground_station and align_heading and airframe)):
        raise ValueError('mission requires released fixture, real GCS, heading, airframe contract, and sole controller')
    mission_config=json.loads(mission_settings.read_text()) if mission_settings else None
    if inspect_hover and inspect_offboard_disarmed:
        raise ValueError('Hover inspection and disarmed Offboard inspection are mutually exclusive')
    if inspect_offboard_disarmed and not (bootstrap_fixture and ground_station and align_heading):
        raise ValueError('Disarmed Offboard inspection requires bootstrap fixture, real GCS and heading alignment')
    if bootstrap_fixture and not (diagnostic_rig and external_vision and with_px4):
        raise ValueError('Bootstrap fixture is disarmed VIO/PX4 validation only')
    if diagnostic_rig and not 0<diagnostic_rig_amplitude<=1.:
        raise ValueError('diagnostic fixture amplitude must be in (0, 1] m')
    if inspect_hover and not (bootstrap_fixture and ground_station):
        raise ValueError('Read-only hover preflight inspection requires released aircraft and a real GCS')
    if external_vision and (not with_px4 or not vio_config):
        raise ValueError("External vision validation requires PX4 and real VIO")
    if diagnostic_rig and ((with_px4 and not external_vision) or not vio_config):
        raise ValueError("Diagnostic rig requires shadow VIO and no PX4")
    course_model=course_model_for_scene(project,scene)
    if output.exists():
        raise FileExistsError("choose a new output directory to preserve evidence")
    output.mkdir(parents=True)
    snapshot_hashes = {}
    for relative in ('sim/gazebo_smoke.py', 'tools/ros_probe.py', 'sim/snapshots.py',
                     'tools/ev_bridge.py', 'dronebench/vision_odometry.py', 'dronebench/telemetry.py',
                     'sim/sensor_rig.py', 'sim/bootstrap_fixture.py', 'dronebench/heading_alignment.py',
                     'tools/hover_qualification.py', 'dronebench/flight_guard.py',
                     'tools/disarmed_offboard_probe.py', 'dronebench/disarmed_offboard.py',
                     'dronebench/causal_telemetry.py','sim/sky_beta_flight_model.py','sim/sky_beta_model.py',
                     'tools/course_flight.py','dronebench/course_mission.py','dronebench/teammate_policy.py',
                     'dronebench/vio_quality.py',
                     'sim/yaw_diagnostic.py',
                     'sim/extend_diagnostic_rail.py','dronebench/depth_recording.py',
                     'sim/runtime/4099_gz_sky_beta_estimated','sim/mission_observer.py',
                     'sim/scene_evidence.py','sim/course_net_visual.py'):
        source_file = project / relative
        if source_file.exists():
            snapshot = output / 'source' / relative
            snapshot.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_file, snapshot)
            snapshot_hashes[relative] = sha256(snapshot.read_bytes()).hexdigest()
    settings = output / 'settings'
    settings.mkdir()
    if airframe_contract:
        shutil.copy2(airframe_contract,settings/'airframe-contract.json')
    if mission_settings:
        shutil.copy2(mission_settings,settings/'mission.json')
    mission_assets={}
    for name,path in [('trajectory.csv',mission_trajectory),('policy.onnx',mission_policy)]:
        if path:
            shutil.copy2(path,settings/name)
            mission_assets[name]=sha256((settings/name).read_bytes()).hexdigest()
    if mission_trajectory:
        for name in ('plan.json','clearance.json','whole-gates.json'):
            path=mission_trajectory.parent/name
            if path.is_file():shutil.copy2(path,settings/('reference-'+name))
    if mission_repo:
        for relative in ('drl/config.py','drl/course.py','drl/obs.py','deploy/runner.py',
                         'config/course_v3.yaml','course/dronehacks-course-v3.obj'):
            source=mission_repo/relative;target=settings/'teammate'/relative
            target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
            mission_assets[relative]=sha256(target.read_bytes()).hexdigest()
    shutil.copy2(scene, settings / 'world.sdf')
    shutil.copy2(course_model,settings/'course-model.sdf')
    if (course_model.parent/'appearance-contract.json').exists():
        shutil.copy2(course_model.parent/'appearance-contract.json',settings/'appearance-contract.json')
    if vio_config:
        for name in ('estimator_config.yaml', 'kalibr_imucam_chain.yaml', 'kalibr_imu_chain.yaml'):
            shutil.copy2(vio_config.parent / name, settings / name)
    environment = dict(os.environ)
    environment.update({"GZ_PARTITION": f"dronehacks_smoke_{os.getpid()}",
                        "ROS_DOMAIN_ID": "42"})
    if hardware_rendering:
        environment.pop("LIBGL_ALWAYS_SOFTWARE", None)
    else:
        environment["LIBGL_ALWAYS_SOFTWARE"] = "1"
    environment["GZ_SIM_RESOURCE_PATH"] = ":".join([
        str(project / "artifacts/course-v3-r6/models"),
        str(scene.parent.parent),
        str(runtime / "PX4-Autopilot/Tools/simulation/gz/models"),
        environment.get("GZ_SIM_RESOURCE_PATH", ""),
    ])
    children, logs = [], []
    child_names = {}
    rootfs = None
    offboard_probe_process = None
    mission_process=None
    mission_observed_exit_wall=None

    def launch(name, command, extra_env=None):
        handle = (output / f"{name}.log").open("wb")
        logs.append(handle)
        child = subprocess.Popen(command, env=dict(environment, **(extra_env or {})), stdout=handle,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child)
        child_names[child.pid] = name
        return child

    result = {"kind": "gazebo_ros_sensor_smoke_not_flight", "commands_sent": False,
              'source_snapshot_sha256': snapshot_hashes,
              "software_rendering_requested": not hardware_rendering, "ros_domain": 42,
              "px4_started": False, "flight_ready": False,
              "script_sha256": sha256(Path(__file__).read_bytes()).hexdigest(),
              "observer_sha256": sha256((project / "tools/ros_probe.py").read_bytes()).hexdigest(),
              "world_sha256": sha256(scene.read_bytes()).hexdigest(),
              "course_model_sha256": sha256(course_model.read_bytes()).hexdigest()}
    result['mission_asset_sha256']=mission_assets
    if diagnostic_rig:
        result.update(kind="constrained_sensor_rig_not_flight", commands_sent=True,
                      flight_commands_sent=False, truth_pose_input_to_estimator=False)
    if bootstrap_fixture:
        result['kind'] = 'disarmed_aircraft_bootstrap_not_flight'
    if inspect_offboard_disarmed:
        result.update(kind='disarmed_offboard_mode_qualification_not_flight',
                      disarmed_offboard_probe_requested=True, flight_commands_sent=False)
    try:
        server = launch("gazebo", ["gz", "sim", "-r", "-s", "--headless-rendering", "-v", "3", str(scene)])
        print("Started headless Gazebo; waiting for sensor discovery.", flush=True)
        topic_names = set()
        for attempt in range(10):
            time.sleep(2)
            if server.poll() is not None:
                raise RuntimeError(f"Gazebo exited with code {server.returncode}; inspect gazebo.log")
            listing = subprocess.run(["gz", "topic", "-l"], env=environment,
                                     capture_output=True, text=True, timeout=8)
            topic_names = set(listing.stdout.splitlines())
            (output / "gazebo-topics.txt").write_text(listing.stdout, encoding="utf-8")
            if "/dronehacks/left" in topic_names:
                break
        if mission_settings or diagnostic_rig:
            # Independent diagnostic capture only. Never bridge this to ROS or
            # provide it to VIO, the mission adapter, or a navigation policy.
            truth_topic='/world/dronehacks/dynamic_pose/info'
            if truth_topic not in topic_names:
                raise RuntimeError('independent scene recording topic unavailable')
            launch('scene-poses',['gz','topic','-e','--json-output','-t',truth_topic])
            result['independent_scene_pose_capture']='scene-poses.log'
            result['scene_pose_used_by_navigation']=False
        if with_px4:
            if ground_station:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as check:
                    check.bind(('127.0.0.1', 14550))
                profile = output / 'qgc-profile'
                qgc_settings = profile / 'config/QGroundControl.org/QGroundControl.ini'
                qgc_settings.parent.mkdir(parents=True)
                qgc_settings.write_text('[AutoConnect]\nautoConnectUDP=true\nudpListenPort=14550\n'
                    'autoConnectPixhawk=false\nautoConnectSiKRadio=false\nautoConnectRTKGPS=false\n'
                    'autoConnectLibrePilot=false\nautoConnectZeroConf=false\nnmeaSource=0\n')
                qgc_process = launch('qgroundcontrol', [str(ground_station)], {
                    'QT_QPA_PLATFORM': 'offscreen', 'QT_QUICK_BACKEND': 'software',
                    'XDG_CONFIG_HOME': str(profile / 'config'), 'XDG_DATA_HOME': str(profile / 'data'),
                    'XDG_CACHE_HOME': str(profile / 'cache')})
                result['ground_station'] = 'actual portable QGroundControl with isolated UDP-only profile'
            # Never attach a second agent to an occupied local transport.
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as check:
                check.bind(("127.0.0.1", 8888))
            prefix = runtime / "install"
            environment["LD_LIBRARY_PATH"] = str(prefix / "lib") + ":" + environment.get("LD_LIBRARY_PATH", "")
            launch("xrce-agent", [str(prefix / "bin/MicroXRCEAgent"), "udp4", "-p", "8888"])
            environment.update({"PX4_GZ_STANDALONE": "1", "PX4_SIM_MODEL": "gz_x500",
                                "PX4_GZ_MODEL_NAME": px4_model_name,
                                "PX4_GZ_WORLD": "dronehacks", "PX4_SYS_AUTOSTART": "4001",
                                "PX4_PARAM_EKF2_GPS_CTRL": "0",
                                "PX4_PARAM_UXRCE_DDS_SYNCT": "0", "HEADLESS": "1"})
            if airframe is not None:
                autostart=airframe.get('px4_autostart_id',4001)
                if type(autostart) is not int or not 1000<=autostart<=9999:
                    raise ValueError('invalid reviewed SITL airframe profile ID')
                environment['PX4_SYS_AUTOSTART']=str(autostart)
                result['px4_autostart_id']=autostart
                for name,number in airframe['px4_parameters'].items():
                    if (not re.fullmatch(r'CA_AIRFRAME|CA_ROTOR_COUNT|CA_ROTOR[0-3]_(PX|PY|PZ|KM)|SIM_GZ_EC_(MIN|MAX|FUNC)[1-4]|THR_MDL_FAC|MPC_THR_HOVER|EKF2_IMU_POS_[XYZ]',name)
                            or type(number) not in (int,float) or not isfinite(number)):
                        raise ValueError('nonphysical/unreviewed PX4 airframe parameter '+name)
                    environment['PX4_PARAM_'+name]=format(number,'.12g')
                result['physical_airframe_parameters']=airframe['px4_parameters']
                result['estimated_airframe']=airframe.get('estimated_parameters',False)
            if external_vision:
                environment['PX4_PARAM_EKF2_EV_CTRL'] = '15'
            if skybeta_profile:
                # Organizer px4/params/03b_ekf2_vision.params: vision horizontal position + yaw
                # only, measured delay (drone 16: 60 ms), 0.1 m noise, message variances honoured.
                # Sim deviation: EV_CTRL=11 adds vision *height* (bit 1) and HGT_REF=3 makes it the
                # reference, standing in for the drone's rangefinder height (HGT_REF=2): PX4 SITL
                # does not ingest this model's down_range sensor, and with baro height PX4 and the
                # VIO-referenced team node disagreed by 0.12 m within 14 s of navigation (v98 abort).
                environment.update({'PX4_PARAM_EKF2_EV_CTRL': '11', 'PX4_PARAM_EKF2_EV_DELAY': '60',
                                    'PX4_PARAM_EKF2_EVP_NOISE': '0.1', 'PX4_PARAM_EKF2_EVV_NOISE': '0.1',
                                    'PX4_PARAM_EKF2_EVA_NOISE': '0.05', 'PX4_PARAM_EKF2_EV_NOISE_MD': '0',
                                    # Sim-only stand-in for the pilot's RC transmitter (drone: COM_RC_IN_MODE=0, real
                                    # RC). PX4 1.16 arms Altitude/Position only with a valid manual-control source
                                    # (mode_requirements.cpp + modeCheck.cpp "No manual control input"): with
                                    # COM_RC_IN_MODE=4 "stick input disabled" nothing is ever valid, so v89-v97 were
                                    # denied in ALTCTL. 1 = joystick only (SITL default): the pilot emulator's
                                    # --sticks stream (throttle down, sticks centred) is accepted as that source.
                                    'PX4_PARAM_COM_RC_IN_MODE': '1',
                                    # EV_CTRL=9 carries no vision height and GPS is off, so EKF2 needs an explicit
                                    # height reference or it reports "missing data" (v91/v92 never armed). The drone
                                    # uses the rangefinder (HGT_REF=2) with baro backup; baro is the backup choice
                                    # here until PX4 SITL ingests the model's down_range sensor.
                                    'PX4_PARAM_EKF2_HGT_REF': '3', 'PX4_PARAM_EKF2_BARO_CTRL': '1'})
                if skybeta_mag_type is not None:
                    environment['PX4_PARAM_EKF2_MAG_TYPE'] = str(int(skybeta_mag_type))
                result['skybeta_profile_px4'] = ('EKF2_EV_CTRL=11 (drone: 9 + rangefinder height) EV_DELAY=60 EVP/EVV_NOISE=0.1 EVA_NOISE=0.05 HGT_REF=3 '
                                                 f'COM_RC_IN_MODE=1 (sim pilot sticks) MAG_TYPE={"default" if skybeta_mag_type is None else skybeta_mag_type}')
                result['external_vision_fusion_configuration'] = 'horizontal position + height + yaw from actual VIO via organizer px4_odom_bridge; EKF2_EV_CTRL=11 (sim stand-in for EV_CTRL=9 + rangefinder height)'
            build_dir = runtime / "PX4-Autopilot/build/px4_sitl_default"
            result['px4_binary_sha256']=sha256((build_dir/'bin/px4').read_bytes()).hexdigest()
            # PX4 v1.16 startup scripts split an unquoted rootfs path on spaces.
            # Keep only PX4's writable state in the user-owned native Linux tree.
            rootfs = runtime / "runs" / output.name / "rootfs"
            rootfs.mkdir(parents=True, exist_ok=False)
            launch("px4", [str(build_dir / "bin/px4"), "-d", str(build_dir / "etc"),
                           "-w", str(rootfs)])
            result["px4_started"] = True
            result["px4_runtime_state"] = str(rootfs.relative_to(runtime))
            result["gnss_fusion_disabled_for_no_gnss_proxy"] = True
            result["clock_contract"] = "Gazebo simulation time; PX4 DDS wall-clock translation disabled per PX4 ROS2 guide"
            print("Started disarmed PX4 and local DDS agent; no control publisher exists.", flush=True)
        bridges = []

        def bridge(gz_topic, ros_topic, gz_type, ros_type):
            if gz_topic in topic_names:
                bridges.append({"gz_topic_name": gz_topic, "ros_topic_name": ros_topic,
                                "gz_type_name": gz_type, "ros_type_name": ros_type,
                                "direction": "GZ_TO_ROS"})

        bridge("/world/dronehacks/clock", "/clock", "gz.msgs.Clock", "rosgraph_msgs/msg/Clock")
        for name in ("left", "right", "rgb", "ideal_depth", "observer", "down_rgb"):
            ros_topic = f"/stereo/{name}/image" if name in ("left", "right") else f"/camera/{name}/image"
            bridge(f"/dronehacks/{name}", ros_topic, "gz.msgs.Image", "sensor_msgs/msg/Image")
            bridge(f"/dronehacks/{name}/camera_info", ros_topic.replace("/image", "/camera_info"),
                   "gz.msgs.CameraInfo", "sensor_msgs/msg/CameraInfo")
        for topic in topic_names:
            if topic.endswith("/imu") and "imu_sensor" in topic:
                bridge(topic, "/imu/data", "gz.msgs.IMU", "sensor_msgs/msg/Imu")
        bridge("/dronehacks/down_range", "/range/scan", "gz.msgs.LaserScan", "sensor_msgs/msg/LaserScan")
        config = output / "bridge.yaml"
        config.write_text(json.dumps(bridges, indent=2) + "\n", encoding="utf-8")
        if not any(b["ros_type_name"] == "sensor_msgs/msg/Image" for b in bridges):
            raise RuntimeError("no camera image topic discovered; inspect Gazebo logs")
        record_topics=([topic.strip() for topic in record_topic.split(',') if topic.strip()]
                       if isinstance(record_topic,str) else list(record_topic or []))
        if record_image and (not record_topics or len(set(record_topics))!=len(record_topics)):
            raise ValueError('recording requires one or more unique ROS image topics')
        camera_topics={'/stereo/left/image','/stereo/right/image','/camera/rgb/image',
                       '/camera/ideal_depth/image','/camera/observer/image','/camera/down_rgb/image'}
        missing_cameras=set(record_topics)&camera_topics-{b['ros_topic_name'] for b in bridges}
        if record_image and missing_cameras:
            raise RuntimeError('requested recording camera unavailable before mission startup: '+', '.join(sorted(missing_cameras)))
        bridge_process = launch("bridge", ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                                           "--ros-args", "-p", f"config_file:={config}"])
        if vio_config is not None:
            if with_px4 and not external_vision:
                raise ValueError("shadow VIO check must run without PX4")
            result["vio_config_sha256"] = sha256(vio_config.read_bytes()).hexdigest()
            result["vio_mode"] = "disarmed_px4_fusion" if external_vision else "shadow_only_no_px4_bridge"
            if estimator == 'cuvslam':
                # Stereo-only cuVSLAM (as on the organizer's Sky Beta stack), publishing the
                # same /vio_shadow topics; see tools/cuvslam_vo_node.py (consistency gate on).
                cu_lib = Path.home()/'dronehacks-models/venv/lib/python3.12/site-packages/nvidia/cu13/lib'
                cu_args = [str(runtime/'cuvslam-venv/bin/python'), str(project/'tools/cuvslam_vo_node.py'),
                           '--calib', str(vio_config.parent/'kalibr_imucam_chain.yaml'),
                           '--log', str(output/'cuvslam-frames.jsonl')]
                if cuvslam_inertial:
                    # Diagnostic variant: cuVSLAM Inertial mode (stereo + /imu/data); the organizer stack runs stereo-only.
                    cu_args += ['--inertial', str(Path(cuvslam_inertial).resolve())]
                vio_process = launch("cuvslam", cu_args, {'LD_LIBRARY_PATH': str(cu_lib)+':'+environment.get('LD_LIBRARY_PATH', '')})
                result['estimator'] = ('cuvslam_stereo_inertial (PyCuVSLAM Inertial mode) with gyro consistency gate' if cuvslam_inertial
                                       else 'cuvslam_stereo_vo (PyCuVSLAM) with gyro consistency gate')
            else:
                vio_process = launch("openvins", ["ros2", "run", "ov_msckf", "run_subscribe_msckf",
                                     str(vio_config), "--ros-args", "-r", "__ns:=/vio_shadow",
                                     "-p", "use_sim_time:=true", "-p", "multi_threading_pubs:=false",
                                     "-p", "multi_threading_subs:=false"])
                result['estimator'] = 'openvins'
            result["vio_publisher_threading"] = "disabled via upstream option after preserved image publisher startup crash"
        if record_bag:
            # Lossless full-rate sensor log for offline estimator comparison
            # (stereo + IMU + clock). Read-only subscriber; never a flight input.
            launch("rosbag", ["ros2", "bag", "record", "-s", "mcap", "--storage-preset-profile", "zstd_fast",
                              "-o", str(output/"sensor-bag"), "/stereo/left/image", "/stereo/right/image",
                              "/imu/data", "/clock"])
            result["sensor_bag"] = "sensor-bag (mcap/zstd): /stereo/left/image /stereo/right/image /imu/data /clock"
        duration = str(observation_seconds(vio_config=vio_config, inspect_hover=inspect_hover,
                                          inspect_offboard_disarmed=inspect_offboard_disarmed))
        if mission_config:
            duration=str(min(1190,75+int(mission_config['mission']['max_wall_s']))) if execute_mission else '125'
            result.update(kind='approved_simulation_mission' if execute_mission else 'readonly_mission_readiness',
                          flight_execution_requested=execute_mission,approval_reference=approval_reference)
        result['observation_duration_wall_s'] = int(duration)
        skybeta_ws = runtime/'skybeta_ws/install/setup.bash'

        def skybeta_launch(name, cmd):
            return launch(name, ['bash', '-c', f'source {skybeta_ws} && exec {cmd}'])
        if skybeta_profile:
            # Real-drone contract: estimator -> /visual_slam/* (relay) -> organizer px4_odom_bridge
            # -> PX4; team node -> /skybeta/setpoint/* -> organizer flight_interface -> PX4.
            skybeta_launch('skybeta-relay', f'{sys.executable} {project/"tools/skybeta_vio_relay.py"}')
            skybeta_launch('px4-odom-bridge', 'ros2 run oakd_vslam_bringup px4_odom_bridge --ros-args -p use_sim_time:=true')
            skybeta_launch('flight-interface', 'ros2 run skybeta_flight_interface flight_interface --ros-args '
                           '-p use_sim_time:=true -p auto_offboard_on_arm:=true')
            result['vision_publication_mode'] = 'organizer px4_odom_bridge (skybeta profile)'
        elif external_vision:
            launch('vision-validation', [sys.executable, str(project / 'tools/ev_bridge.py'),
                *(['--flight-support','--approval-reference',approval_reference,'--shared-simulation-clock'] if execute_mission else ['--disarmed-validation']),
                '--seconds', str(int(duration)+10), '--output', str(output / 'vision-publication.json'),
                *(['--airframe-contract',str(airframe_contract)] if airframe_contract else []),
                *(['--align-heading-from-px4'] if align_heading else [])])
            result['vision_publication_mode'] = 'approved_bounded_sensor_support' if execute_mission else 'bounded_disarmed_only_not_flight'
        print(f"ROS bridge started; observing actual messages for {duration} wall seconds.", flush=True)
        record_args=([arg for index,topic in enumerate(record_topics)
                      for arg in ("--record-topic",topic,"--record-dir",
                                  str(output/('rgb-sequence' if index==0 else 'image-sequence-'+str(index+1))))]
                     if record_image else [])
        if record_vio_odometry:
            record_args.extend(['--record-odometry-topic','/vio_shadow/odomimu',
                                '--odometry-output',str(output/'vio-odometry.jsonl'),
                                '--record-feature-topic','/vio_shadow/points_slam',
                                '--feature-output',str(output/('vio-feature-clouds.jsonl' if record_all_vio_features else 'vio-slam-features.jsonl'))])
            if record_all_vio_features:
                record_args.extend(['--record-feature-topic','/vio_shadow/points_msckf'])
        if record_ideal_depth:
            record_args.extend(['--record-depth-topic','/camera/ideal_depth/image',
                '--depth-dir',str(output/'ideal-depth-evidence'),'--depth-units','metres',
                '--depth-provenance','rendered_ideal_depth_geometry_diagnostic_not_stereo_or_flight_input'])
            result['ideal_depth_diagnostic_only']=True
        observation = subprocess.Popen([sys.executable, str(project / "tools/ros_probe.py"),
                                      "--seconds", duration, *([] if execute_mission else ['--require-disarmed']),
                                      *(['--stop-file',str(output/'stop-observation')] if mission_settings else []),
                                      "--snapshot-dir", str(output / "snapshots"),
                                      "--output", str(output / "ros-observation.json"), *record_args],
                                     env=environment, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, text=True, start_new_session=True)
        children.append(observation)
        if mission_observe_at_start:
            optional=[]
            for flag,path in [('--repo',mission_repo),('--policy',mission_policy),('--trajectory',mission_trajectory)]:
                if path: optional.extend([flag,str(path)])
            mission_process=launch('course-mission',[sys.executable,str(project/'tools/course_flight.py'),
                '--settings',str(mission_settings),'--airframe-contract',str(airframe_contract),
                '--output',str(output/'course-mission.json'),'--observe-seconds',str(mission_observe_seconds),
                *optional])
        deadline = time.monotonic() + int(duration) + 20
        rig_started = time.monotonic()
        rig_plan=(((15,diagnostic_rig_amplitude),(25,0.),(35,diagnostic_rig_amplitude),(45,0.),(60,None))
                  if diagnostic_rig_amplitude>.15 else
                  ((15,.12),(30,-.12),(45,0.),(60,None)))
        if diagnostic_yaw_rad>.6 or diagnostic_long_profile:
            rig_plan=((15,diagnostic_rig_amplitude),(35,0.),
                      (55,diagnostic_rig_amplitude),(75,0.),(100,None))
        rig_steps = iter(rig_plan if bootstrap_fixture else
                         ((15, .12), (30, -.12), (45, 0.)))
        next_rig_step = next(rig_steps) if diagnostic_rig else None
        result["diagnostic_rig_commands"] = []
        yaw_times=(20,55,80,95) if diagnostic_yaw_rad>.6 or diagnostic_long_profile else (20,30,40,50)
        yaw_targets=(diagnostic_yaw_rad,0.,diagnostic_yaw_rad,0.) if diagnostic_yaw_rad>.6 or diagnostic_long_profile else (
            diagnostic_yaw_rad,-diagnostic_yaw_rad,diagnostic_yaw_rad,0.)
        yaw_steps=iter(zip(yaw_times,yaw_targets))
        next_yaw_step=next(yaw_steps) if diagnostic_yaw_rad else None
        result['diagnostic_yaw_commands']=[]
        while True:
            if next_yaw_step and time.monotonic()-rig_started>=next_yaw_step[0]:
                yaw_sent=subprocess.run(['gz','topic','-t','/diagnostic/rig_yaw_position',
                    '-m','gz.msgs.Double','-p',f'data: {next_yaw_step[1]}'],
                    env=environment,capture_output=True,text=True,timeout=8)
                result['diagnostic_yaw_commands'].append({
                    'offset_wall_s':time.monotonic()-rig_started,
                    'target_rad':next_yaw_step[1],'exit_code':yaw_sent.returncode})
                if yaw_sent.returncode:
                    raise RuntimeError('Diagnostic yaw command failed')
                next_yaw_step=next(yaw_steps,None)
            if next_rig_step and time.monotonic() - rig_started >= next_rig_step[0]:
                command = (["gz", "topic", "-t", "/diagnostic/bootstrap_detach", "-m", "gz.msgs.Empty", "-p", ""]
                           if next_rig_step[1] is None else
                           ["gz", "topic", "-t", "/diagnostic/rig_position", "-m", "gz.msgs.Double",
                            "-p", f"data: {next_rig_step[1]}"])
                sent = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=8)
                result["diagnostic_rig_commands"].append({"offset_wall_s": time.monotonic() - rig_started,
                    "target_m": next_rig_step[1], "exit_code": sent.returncode})
                if sent.returncode:
                    raise RuntimeError("Diagnostic rig command failed")
                if next_rig_step[1] is None and inspect_hover:
                    launch('hover-preflight-readonly', [sys.executable, str(project / 'tools/hover_qualification.py'),
                        '--shared-simulation-clock', '--observe-seconds', '35',
                        '--output', str(output / 'hover-preflight.json')])
                if next_rig_step[1] is None and inspect_offboard_disarmed:
                    offboard_probe_process = launch('offboard-disarmed-probe',
                        [sys.executable, str(project / 'tools/disarmed_offboard_probe.py'),
                         '--probe-mode', '--shared-simulation-clock',
                         '--output', str(output / 'offboard-disarmed-preflight.json')])
                    result['disarmed_offboard_probe_start_offset_wall_s'] = time.monotonic()-rig_started
                if next_rig_step[1] is None and skybeta_profile:
                    skybeta_launch('team-nav', f'{sys.executable} {project/"tools/skybeta_team_nav.py"} '
                                   f'--trajectory {mission_trajectory} --journal {output/"team-nav.jsonl"} '
                                   + ' '.join(team_nav_args))
                    # --sticks: SIM ONLY pilot transmitter stand-in (see tools/skybeta_pilot_emulator.py).
                    mission_process = skybeta_launch('pilot-emulator', f'{sys.executable} {project/"tools/skybeta_pilot_emulator.py"} '
                                   f'--output {output/"course-mission.json"} --max-wall-s {int(mission_config["mission"]["max_wall_s"])} --sticks')
                elif next_rig_step[1] is None and native_program:
                    # A FlyX-lab style main.py (native /fmu/in Offboard, arm, land) instead of our
                    # supervisor; pass/fail is its exit code (0 = landed + disarmed after the mission).
                    _np = shlex.split(str(native_program))
                    mission_process = launch('native-program', [sys.executable, '-u', str(Path(_np[0]).resolve()), *_np[1:]])
                    result['native_program'] = str(native_program)
                elif next_rig_step[1] is None and mission_settings and not mission_observe_at_start:
                    optional=[]
                    for flag,path in [('--repo',mission_repo),('--policy',mission_policy),('--trajectory',mission_trajectory)]:
                        if path: optional.extend([flag,str(path)])
                    mission_process=launch('course-mission',[sys.executable,str(project/'tools/course_flight.py'),
                        '--settings',str(mission_settings),'--airframe-contract',str(airframe_contract),
                        '--output',str(output/'course-mission.json'),'--shared-simulation-clock',
                        *(['--execute','--approval-reference',approval_reference] if execute_mission else []),*optional])
                next_rig_step = next(rig_steps, None)
            if mission_process is not None and mission_process.poll() is not None:
                if mission_observed_exit_wall is None: mission_observed_exit_wall=time.monotonic()
                if time.monotonic()-mission_observed_exit_wall>=2:
                    (output/'stop-observation').touch(exist_ok=True)
            try:
                stdout, stderr = observation.communicate(timeout=1)
                break
            except subprocess.TimeoutExpired:
                if vio_config is not None and vio_process.poll() is not None:
                    result["vio_process_exit_during_observation"] = vio_process.returncode
                    raise RuntimeError("OpenVINS exited; inspect openvins.log")
                if ground_station and qgc_process.poll() is not None:
                    raise RuntimeError('Actual ground station exited; inspect qgroundcontrol.log')
                if time.monotonic() > deadline:
                    raise RuntimeError("bounded ROS observation timed out")
        (output / "probe.log").write_text(stdout + stderr, encoding="utf-8")
        if observation.returncode:
            raise RuntimeError(f"ROS probe failed with code {observation.returncode}")
        data = json.loads((output / "ros-observation.json").read_text())
        if vio_config is not None:
            result["vio_process_exit_during_observation"] = vio_process.poll()
            result["vio_odometry_observed"] = data["streams"].get("/vio_shadow/odomimu", {})
            if vio_process.poll() is not None:
                raise RuntimeError("OpenVINS exited; inspect openvins.log")
        images = {name: stream for name, stream in data["streams"].items()
                  if stream["type"] == "sensor_msgs/msg/Image"}
        result["observed_images"] = {name: stream["messages"] for name, stream in images.items()}
        expected = {"/stereo/left/image", "/stereo/right/image", "/camera/rgb/image", "/camera/ideal_depth/image"}
        result["missing_images"] = sorted(expected - images.keys())
        result["sensor_streams_observed"] = not result["missing_images"] and all(
            stream["source_clock_observed_advancing"] for stream in images.values())
        result["clock_and_imu_observed"] = all(
            data["streams"].get(topic, {}).get("source_clock_observed_advancing", False)
            for topic in ("/clock", "/imu/data"))
        result["bridge_exited"] = bridge_process.poll()
        if not result["sensor_streams_observed"] or not result["clock_and_imu_observed"]:
            raise RuntimeError("no advancing image streams observed; not a passing smoke test")
        if with_px4:
            wanted = {"px4_msgs/msg/VehicleStatus", "px4_msgs/msg/VehicleLocalPosition",
                      "px4_msgs/msg/VehicleLandDetected"}
            observed = {s["type"] for s in data["streams"].values() if s["messages"] > 0}
            result["missing_px4_message_types"] = sorted(wanted - observed)
            if result["missing_px4_message_types"]:
                raise RuntimeError("required PX4 telemetry missing; inspect PX4/DDS logs")
        # Simulator pose is isolated debug evidence for scene assembly only.
        # It is never bridged into ROS odometry or consumed by a planner.
        try:
            poses = subprocess.run(["gz", "topic", "-e", "-n", "1", "--json-output", "-t",
                                    "/world/dronehacks/dynamic_pose/info"], env=environment,
                                   capture_output=True, text=True, timeout=5)
            (output / "debug-scene-poses.json").write_text(poses.stdout, encoding="utf-8")
        except subprocess.TimeoutExpired:
            result["scene_pose_debug"] = "timed out; not used for flight or estimator checks"
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
            # Stop the estimator while bridge and simulation clocks still run.
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        for handle in logs:
            handle.close()
        result["all_children_stopped"] = all(child.poll() is not None for child in children)
        result["child_exit_codes"] = {child_names.get(c.pid, "observation"): c.returncode for c in children}
        if native_program:
            code = mission_process.returncode if mission_process else None
            result['native_program_exit_code'] = code
            result['mission_controller_completed_landed_disarmed'] = code == 0
            result['independent_course_grade'] = False
            if code != 0:
                result['mission_error'] = f'native program exit code {code} (0 = landed + disarmed after its mission)'
        elif mission_settings:
            try:
                mission_report=json.loads((output/'course-mission.json').read_text())
                result['mission_report']=mission_report
                result['flight_commands_sent']='arm' in mission_report.get('commands_published',[])
                checked=(mission_report.get('state')=='COMPLETE' if skybeta_profile else
                         mission_result_passed(mission_report,mission_process.returncode if mission_process else None,execute_mission))
                result['mission_readonly_health_passed']=checked if not execute_mission else None
                result['mission_controller_completed_landed_disarmed']=checked if execute_mission else False
                result['independent_course_grade']=False
                if not checked:
                    result['mission_error']='Requested mission did not establish its required readonly-health or landed/disarmed outcome'
            except (OSError,ValueError) as exc:
                result['mission_error']=str(exc)
        if inspect_offboard_disarmed:
            probe_path = output / 'offboard-disarmed-preflight.json'
            exit_code = offboard_probe_process.returncode if offboard_probe_process is not None else None
            try:
                probe_report = json.loads(probe_path.read_text(encoding='utf-8'))
                result['disarmed_offboard_probe'] = {'report': probe_report,
                    'sha256': sha256(probe_path.read_bytes()).hexdigest(), 'exit_code': exit_code}
                result['disarmed_offboard_qualification_passed'] = disarmed_offboard_passed(probe_report, exit_code)
            except (OSError, ValueError, TypeError, AttributeError) as exc:
                result['disarmed_offboard_probe'] = {'error': str(exc), 'exit_code': exit_code}
                result['disarmed_offboard_qualification_passed'] = False
            if not result['disarmed_offboard_qualification_passed']:
                result['mode_qualification_error'] = 'Disarmed Offboard mode/preflight diagnostic did not qualify; sensor delivery is separate'
        if vio_config and result["child_exit_codes"].get("cuvslam" if estimator == "cuvslam" else "openvins") != 0:
            result["shutdown_error"] = "OpenVINS did not exit cleanly; sensor delivery is not runtime qualification"
        if rootfs is not None:
            try:
                exported = []
                for path in rootfs.glob("log/**/*.ulg"):
                    destination = output / "px4-logs" / path.name
                    destination.parent.mkdir(exist_ok=True)
                    shutil.copy2(path, destination)
                    exported.append({"file": str(destination.relative_to(output)),
                                     "sha256": sha256(destination.read_bytes()).hexdigest()})
                result["px4_logs"] = exported
            except OSError as exc:
                result["log_export_error"] = str(exc)
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2), flush=True)
    if result.get("shutdown_error"):
        raise RuntimeError(result["shutdown_error"])
    if result.get('mode_qualification_error'):
        raise RuntimeError(result['mode_qualification_error'])
    if result.get('mission_error'):
        raise RuntimeError(result['mission_error'])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--with-px4", action="store_true")
    parser.add_argument("--vio-config", type=Path)
    parser.add_argument("--record-image", action="store_true")
    parser.add_argument("--diagnostic-rig", action="store_true")
    parser.add_argument("--external-vision", action="store_true")
    parser.add_argument("--bootstrap-fixture", action="store_true")
    parser.add_argument("--ground-station", type=Path)
    inspection = parser.add_mutually_exclusive_group()
    inspection.add_argument("--inspect-hover", action="store_true")
    inspection.add_argument("--inspect-offboard-disarmed", action="store_true")
    parser.add_argument("--align-heading", action="store_true")
    parser.add_argument('--airframe-contract',type=Path)
    parser.add_argument('--mission-settings',type=Path)
    parser.add_argument('--execute-mission',action='store_true')
    parser.add_argument('--mission-observe-at-start',action='store_true',
                        help='launch the readonly mission telemetry/quality observer before diagnostic rail motion')
    parser.add_argument('--mission-observe-seconds',type=float,default=40.)
    parser.add_argument('--approval-reference')
    parser.add_argument('--mission-repo',type=Path)
    parser.add_argument('--mission-policy',type=Path)
    parser.add_argument('--mission-trajectory',type=Path)
    parser.add_argument('--record-topic',default='/camera/rgb/image')
    parser.add_argument('--estimator',choices=['openvins','cuvslam'],default='openvins')
    parser.add_argument('--native-program',type=str,default=None,help='run this FlyX-lab style main.py at rig detach instead of the course supervisor')
    parser.add_argument('--cuvslam-inertial',type=Path,default=None,help='Kalibr IMU yaml: run cuVSLAM in Inertial mode (diagnostic; organizer stack is stereo-only)')
    parser.add_argument('--skybeta-profile',action='store_true',help='organizer flight_interface + px4_odom_bridge + team_nav (sim pilot)')
    parser.add_argument('--team-nav-arg',action='append',default=[],help='extra argument for tools/skybeta_team_nav.py')
    parser.add_argument('--skybeta-mag-type',type=int,default=None,help='override EKF2_MAG_TYPE in the skybeta profile (sim diagnostic)')
    parser.add_argument('--record-bag',action='store_true',help='lossless mcap of stereo+IMU+clock for offline estimator benchmarks')
    parser.add_argument('--record-vio-odometry',action='store_true',
                        help='save read-only /vio_shadow/odomimu samples for offline estimator review')
    parser.add_argument('--record-all-vio-features',action='store_true',
                        help='record both SLAM and MSCKF clouds with explicit topic labels')
    parser.add_argument('--record-ideal-depth',action='store_true',
                        help='record rendered ideal depth for disarmed offline geometry diagnosis only')
    parser.add_argument('--long-fixture-profile',action='store_true',help='disarmed motion holds fixture until 100 wall seconds')
    parser.add_argument('--diagnostic-rig-amplitude',type=float,default=.12,
                        help='bounded fixture displacement in meters; applies only to disarmed rig motion')
    parser.add_argument('--diagnostic-yaw-rad',type=float,default=0.,
                        help='bounded physical yaw oscillation; disarmed fixture only')
    parser.add_argument("--px4-model-name", default='dronehacks_x500_proxy')
    parser.add_argument("--hardware-rendering", action="store_true",
                        help="allow GPU rendering; verify actual renderer in Gazebo logs")
    args = parser.parse_args()
    run(Path(__file__).resolve().parents[1], Path.home() / "dronehacks-runtime",
        args.scene.resolve(), args.output.resolve(), with_px4=args.with_px4,
        vio_config=args.vio_config.resolve() if args.vio_config else None,
        hardware_rendering=args.hardware_rendering, record_image=args.record_image,
        diagnostic_rig=args.diagnostic_rig, external_vision=args.external_vision,
        px4_model_name=args.px4_model_name, bootstrap_fixture=args.bootstrap_fixture,
        ground_station=args.ground_station.resolve() if args.ground_station else None,
        inspect_hover=args.inspect_hover, align_heading=args.align_heading,
        inspect_offboard_disarmed=args.inspect_offboard_disarmed,
        airframe_contract=args.airframe_contract.resolve() if args.airframe_contract else None,
        mission_settings=args.mission_settings.resolve() if args.mission_settings else None,
        execute_mission=args.execute_mission,approval_reference=args.approval_reference,
        mission_repo=args.mission_repo,mission_policy=args.mission_policy,mission_trajectory=args.mission_trajectory,
        record_topic=args.record_topic,record_vio_odometry=args.record_vio_odometry,record_bag=args.record_bag,estimator=args.estimator,skybeta_profile=args.skybeta_profile,team_nav_args=tuple(args.team_nav_arg),skybeta_mag_type=args.skybeta_mag_type,cuvslam_inertial=args.cuvslam_inertial,native_program=args.native_program,
        mission_observe_at_start=args.mission_observe_at_start,
        mission_observe_seconds=args.mission_observe_seconds,
        diagnostic_rig_amplitude=args.diagnostic_rig_amplitude,
        diagnostic_yaw_rad=args.diagnostic_yaw_rad,record_all_vio_features=args.record_all_vio_features,
        record_ideal_depth=args.record_ideal_depth,diagnostic_long_profile=args.long_fixture_profile)
