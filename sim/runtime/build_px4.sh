#!/usr/bin/env bash
# Local proxy build only; does not start PX4, Gazebo, DDS or a mission.
set -euo pipefail
test "$(id -u)" != 0
runtime_dir="$HOME/dronehacks-runtime"
mkdir -p "$runtime_dir"
cd "$runtime_dir"
if [ ! -d PX4-Autopilot ]; then
    git clone --depth 1 --branch v1.16.0 https://github.com/PX4/PX4-Autopilot.git
fi
cd PX4-Autopilot
test "$(git describe --tags --exact-match)" = v1.16.0
test -z "$(git status --porcelain --untracked-files=no)"
git submodule update --init --recursive --depth 1 \
    src/modules/mavlink/mavlink src/lib/events/libevents \
    src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client \
    src/drivers/gps/devices src/lib/heatshrink/heatshrink Tools/simulation/gz
if [ ! -d "$runtime_dir/px4-venv" ]; then
    python3 -m venv --system-site-packages "$runtime_dir/px4-venv"
fi
source "$runtime_dir/px4-venv/bin/activate"
export PIP_PROGRESS_BAR=off
python -m pip install -r Tools/setup/requirements.txt
make -j2 px4_sitl_default
printf '\nPX4 build complete; no autopilot started.\n'
git rev-parse HEAD
git submodule status
python -m pip freeze > "$runtime_dir/px4-python-lock.txt"
