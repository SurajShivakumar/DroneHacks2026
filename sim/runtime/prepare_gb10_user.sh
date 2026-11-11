#!/usr/bin/env bash
# User-owned preparation; no sudo, simulator or GPU work.
set -euo pipefail
cd "$HOME/dronehacks-2026-gb10"
tar --keep-old-files -xzf transfer-v1.tar.gz
python3 - <<'PY'
from pathlib import Path
for path in Path('sim/runtime').glob('*.sh'):
    path.write_bytes(path.read_bytes().replace(b'\r\n', b'\n'))
PY
mkdir -p "$HOME/dronehacks-runtime"
cd "$HOME/dronehacks-runtime"
if [ ! -d PX4-Autopilot ]; then
    git clone --depth 1 --branch v1.16.0 https://github.com/PX4/PX4-Autopilot.git
fi
test "$(git -C PX4-Autopilot rev-parse HEAD)" = 6ea3539157ca358c70a515878b77077af7d4611d
git -C PX4-Autopilot submodule update --init --depth 1 Tools/simulation/gz
cd "$HOME/dronehacks-2026-gb10"
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --disable-pip-version-check pytest numpy Pillow
.venv/bin/python -m pytest tests/ -q
printf '\nUser preparation complete. Admin dependency installation remains separate.\n'
