#!/usr/bin/env bash
set -euo pipefail
cd "$HOME/dronehacks-2026-gb10"
python3 tools/encode_recording.py artifacts/gb10-vio-rig-v3/rgb-sequence artifacts/gb10-vio-rig-v3/video
