#!/usr/bin/env bash
# Run a command inside the WSL cuVSLAM venv with the CUDA 13 math libs on the loader path.
# Usage: bash tools/wsl/cuvslam_env.sh python tools/benchmark_cuvslam.py run ...
. ~/cuvslam-venv/bin/activate
export LD_LIBRARY_PATH="$HOME/cuvslam-venv/lib/python3.12/site-packages/nvidia/cu13/lib:/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}"
cd "$(dirname "$0")/../.."
exec "$@"
