#!/usr/bin/env bash
# Local source checkout only. Does not launch a simulator or flight process.
set -euo pipefail
test "$(id -u)" != 0
runtime_dir="$HOME/dronehacks-runtime"
mkdir -p "$runtime_dir/ros_ws/src"
cd "$runtime_dir"
if [ ! -d PX4-Autopilot ]; then
    git clone --depth 1 --branch v1.16.0 https://github.com/PX4/PX4-Autopilot.git
fi
test "$(git -C PX4-Autopilot describe --tags --exact-match)" = v1.16.0
git -C PX4-Autopilot submodule update --init --recursive --depth 1 \
    src/modules/mavlink/mavlink src/lib/events/libevents \
    src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client \
    src/drivers/gps/devices src/lib/heatshrink/heatshrink Tools/simulation/gz
if [ ! -d Micro-XRCE-DDS-Agent ]; then
    git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
fi
if [ ! -d ros_ws/src/px4_msgs ]; then
    git clone --depth 1 --branch release/1.16 https://github.com/PX4/px4_msgs.git ros_ws/src/px4_msgs
fi
python3 - <<'PY'
from pathlib import Path
import json, subprocess
root=Path.cwd()
repos=['PX4-Autopilot','Micro-XRCE-DDS-Agent','ros_ws/src/px4_msgs']
lock={repo:subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip() for repo in repos}
path=root/'source-lock.json'
if path.exists() and json.loads(path.read_text()) != lock:
    raise RuntimeError('Existing source lock differs; review instead of replacing it')
path.write_text(json.dumps(lock,indent=2)+'\n')
print(json.dumps(lock,indent=2))
PY
