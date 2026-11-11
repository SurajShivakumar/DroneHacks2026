#!/usr/bin/env bash
# Read-only graphics diagnostics. No host permissions or jobs changed.
set -euo pipefail
id
ls -l /dev/dri /dev/nvidia0 /dev/nvidiactl
python3 - <<'PY'
from pathlib import Path
for root in (Path.home()/'.gz', Path.home()/'.ignition'):
    if not root.exists():
        continue
    for path in root.rglob('*ogre*.log'):
        print('RENDER LOG', path)
        lines=path.read_text(errors='replace').splitlines()
        for line in lines:
            if any(word in line.lower() for word in ('renderer', 'vendor', 'opengl version', 'llvmpipe', 'nvidia')):
                print(line[:300])
PY
