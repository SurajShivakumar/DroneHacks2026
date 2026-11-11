#!/usr/bin/env bash
# Sensor-only estimated Sky Beta; no PX4, no arm/thrust commands.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$HOME/dronehacks-2026-gb10"
python3 - <<'PY'
from pathlib import Path
from zipfile import ZipFile
root=Path('artifacts').resolve()
assert not (root/'sky-beta-estimated-v2').exists(), 'Preserve an existing model package'
with ZipFile(root/'sky-beta-flight-transfer-v1.zip') as archive:
    for member in archive.infolist():
        if not member.filename.startswith('sky-beta-estimated-v2/') or not (root/member.filename).resolve().is_relative_to(root):
            raise ValueError('unexpected asset archive member')
    archive.extractall(root)
PY
python3 -m sim.gazebo_smoke --scene artifacts/sky-beta-estimated-v2/world.sdf \
    --airframe-contract artifacts/sky-beta-estimated-v2/airframe-contract.json \
    --px4-model-name sky_beta_estimated --hardware-rendering --record-image \
    --output artifacts/gb10-sky-beta-sensors-v1
