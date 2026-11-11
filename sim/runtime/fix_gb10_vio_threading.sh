#!/usr/bin/env bash
# Respect upstream subscriber threading setting; preserve estimator readiness.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$HOME/dronehacks-runtime/vio_ws"
python3 - <<'PY'
from pathlib import Path
p=Path('src/open_vins/ov_msckf/src/run_subscribe_msckf.cpp')
text=p.read_text()
old='  params.use_multi_threading_subs = true;'
new='  // Honor parsed multi_threading_subs; detached workers can outlive shutdown.'
if old not in text and new not in text:
    raise RuntimeError('Unexpected source; review threading override')
p.write_text(text.replace(old,new))
PY
git -C src/open_vins diff --check
mkdir -p build/foreground-threading-v1
git -C src/open_vins diff > build/foreground-threading-v1/native.patch
export MAKEFLAGS='-j2 -l2'
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --packages-select ov_msckf --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON
