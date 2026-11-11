#!/usr/bin/env bash
# User-owned SITL runtime profile; never changes a hardware autopilot.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
from hashlib import sha256
p=Path.home()/'dronehacks-2026-gb10/sim/runtime/4099_gz_sky_beta_estimated'
target=Path.home()/'dronehacks-runtime/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/4099_gz_sky_beta_estimated'
data=p.read_bytes().replace(b'\r\n',b'\n')
if target.exists() and target.read_bytes()!=data:
    raise ValueError('Do not overwrite an unrelated existing SITL profile')
target.write_bytes(data)
target.chmod(0o755)
print('Installed simulation-only profile',sha256(data).hexdigest())
PY
