#!/usr/bin/env bash
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
import json
root=Path.home()/'dronehacks-models/checkpoints/openfly-21dcce'
for name,size in [('model-00001-of-00003.safetensors',6948961960),('model-00002-of-00003.safetensors',6971232040)]:
    p=root/(name+'.upload-v1')
    before=p.stat().st_size
    if before < size:
        safe=max(0,before-4*1024*1024)//1048576*1048576
        with p.open('r+b') as stream:stream.truncate(safe)
        print(json.dumps({'name':name,'previous_partial':before,'resume_at':safe}))
PY
