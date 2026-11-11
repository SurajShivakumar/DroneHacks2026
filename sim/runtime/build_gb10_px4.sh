#!/usr/bin/env bash
# Native ARM64 proxy build. No autopilot, simulator or flight starts.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
runtime_dir="$HOME/dronehacks-runtime"
cd "$runtime_dir/PX4-Autopilot"
test "$(git rev-parse HEAD)" = 6ea3539157ca358c70a515878b77077af7d4611d
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
make -j4 px4_sitl_default
python -m pip freeze > "$runtime_dir/gb10-px4-python-lock.txt"
sha256sum build/px4_sitl_default/bin/px4
