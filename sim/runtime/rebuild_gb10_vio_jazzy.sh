#!/usr/bin/env bash
# Minimal recorded Jazzy API header migration; no estimator or flight starts.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$HOME/dronehacks-runtime/vio_ws"
python3 - <<'PY'
from pathlib import Path
root=Path('src/open_vins')
path=root/'ov_msckf/src/ros/ROS2Visualizer.h'
text=path.read_text()
old='#include <image_transport/image_transport.h>'
new='#include <image_transport/image_transport.hpp>'
if old not in text and new not in text:
    raise RuntimeError('Unexpected upstream header; review before editing')
path.write_text(text.replace(old,new))
for relative in ('ov_msckf/src/ros/ROS2Visualizer.h', 'ov_msckf/src/ros/ROSVisualizerHelper.h'):
    path=root/relative
    text=path.read_text()
    path.write_text(text.replace('<tf2_geometry_msgs/tf2_geometry_msgs.h>',
                                '<tf2_geometry_msgs/tf2_geometry_msgs.hpp>'))
PY
git -C src/open_vins diff --check
git -C src/open_vins diff -- ov_msckf/src/ros/ROS2Visualizer.h ov_msckf/src/ros/ROSVisualizerHelper.h
export CMAKE_BUILD_PARALLEL_LEVEL=4
export MAKEFLAGS=-j4
colcon build --packages-select ov_msckf --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ROS=ON
