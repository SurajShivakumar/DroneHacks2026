#!/usr/bin/env bash
# Isolated laptop fallback; no ROS/system/existing .venv modification.
set -eo pipefail
root="$HOME/dronehacks-models-local"
mkdir -p "$root"
if [ ! -d "$root/venv" ]; then python3 -m venv "$root/venv"; fi
export PIP_PROGRESS_BAR=off
"$root/venv/bin/python" -m pip install torch==2.9.1 torchvision==0.24.1 \
    --index-url https://download.pytorch.org/whl/cu130 --extra-index-url https://pypi.org/simple
"$root/venv/bin/python" -m pip install -r "$PWD/configs/models/requirements-local-nf4.txt"
if [ ! -d "$root/OpenFly-Platform" ]; then
    git clone --depth 1 https://github.com/SHAILAB-IPEC/OpenFly-Platform.git "$root/OpenFly-Platform"
    git -C "$root/OpenFly-Platform" fetch origin c075075497a7122bad82f5b76b9be926ad5a81b3
    git -C "$root/OpenFly-Platform" checkout --detach c075075497a7122bad82f5b76b9be926ad5a81b3
fi
test "$(git -C "$root/OpenFly-Platform" rev-parse HEAD)" = c075075497a7122bad82f5b76b9be926ad5a81b3
"$root/venv/bin/python" -m pip freeze > "$root/dependency-lock.txt"
printf '\nIsolated laptop NF4 environment prepared; no model loaded or control sent.\n'
