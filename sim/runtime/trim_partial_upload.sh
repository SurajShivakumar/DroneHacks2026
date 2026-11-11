#!/usr/bin/env bash
# Discard only an unfinished transfer tail before standard SFTP resume.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
import json
p=Path.home()/'dronehacks-models/checkpoints/openfly-21dcce/model-00001-of-00003.safetensors.upload-v1'
size=p.stat().st_size
safe=max(0,size-4*1024*1024)
safe=safe//(1024*1024)*(1024*1024)
with p.open('r+b') as stream:
    stream.truncate(safe)
print(json.dumps({'file':p.name,'previous_partial_bytes':size,'resume_bytes':safe,'reason':'retransfer unfinished SFTP window tail; final SHA256 still mandatory'}))
PY
