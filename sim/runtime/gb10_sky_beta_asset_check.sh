#!/usr/bin/env bash
# Passive imported Sky Beta rigid body only; no autopilot or thrust.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$HOME/dronehacks-2026-gb10"
python3 - <<'PY'
from pathlib import Path
from zipfile import ZipFile
root=Path('artifacts/sky-beta-models-v2').resolve()
root.mkdir(exist_ok=False)
with ZipFile('artifacts/sky-beta-model-transfer-v2.zip') as archive:
    for member in archive.infolist():
        target=(root/member.filename).resolve()
        if not target.is_relative_to(root):
            raise ValueError('asset archive path escaped intended directory')
    archive.extractall(root)
PY
python3 -m sim.sky_beta_asset_check \
    --model artifacts/sky-beta-models-v2/sky-beta-model-v3 \
    --output artifacts/gb10-sky-beta-passive-v2
