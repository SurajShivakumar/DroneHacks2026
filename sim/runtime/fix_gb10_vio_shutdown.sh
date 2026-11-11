#!/usr/bin/env bash
# Destroy ROS-owned objects before process-static/context teardown.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$HOME/dronehacks-runtime/vio_ws"
python3 - <<'PY'
from pathlib import Path
p=Path('src/open_vins/ov_msckf/src/run_subscribe_msckf.cpp')
text=p.read_text()
old='  viz->visualize_final();'
new='''  viz->visualize_final();
#if ROS_AVAILABLE == 2
  // Executor has stopped: release publishers/subscribers while node is alive.
  viz.reset();
  sys.reset();
#endif'''
if '  viz.reset();' not in text:
    if text.count(old) != 1:
        raise RuntimeError('Unexpected shutdown source')
    p.write_text(text.replace(old,new))
PY
git -C src/open_vins diff --check
git -C src/open_vins diff > build/foreground-threading-v1/native-with-shutdown.patch
export MAKEFLAGS='-j2 -l2'
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --packages-select ov_msckf --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON
