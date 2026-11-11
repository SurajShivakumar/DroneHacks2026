#!/usr/bin/env bash
# Offline: replay a recorded sensor bag through tools/cuvslam_vo_node.py (no sim, no PX4).
# Usage: replay_cuvslam.sh <bag-dir> <out-dir> [extra node args...]
set -eo pipefail
BAG="$1"; OUT="$2"; shift 2
source /opt/ros/jazzy/setup.bash
cd "$HOME/dronehacks-2026-gb10"
export ROS_DOMAIN_ID="${REPLAY_DOMAIN:-77}"
export LD_LIBRARY_PATH="$HOME/dronehacks-models/venv/lib/python3.12/site-packages/nvidia/cu13/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$OUT"
"$HOME/dronehacks-runtime/cuvslam-venv/bin/python" tools/cuvslam_vo_node.py \
  --calib artifacts/vio-feature-budget-v1/kalibr_imucam_chain.yaml --log "$OUT/cuvslam.jsonl" "$@" > "$OUT/node.log" 2>&1 &
NODE=$!
sleep 10
ros2 bag play "$BAG" --clock 100 > "$OUT/play.log" 2>&1 || true
sleep 3
kill -INT $NODE; wait $NODE || true
echo REPLAY_DONE
