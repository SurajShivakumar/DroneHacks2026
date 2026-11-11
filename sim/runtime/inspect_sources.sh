#!/usr/bin/env bash
set -euo pipefail
runtime_dir="$HOME/dronehacks-runtime"
python3 - "$runtime_dir" <<'PY'
from pathlib import Path
import sys
root=Path(sys.argv[1])/'PX4-Autopilot'
files=['Tools/simulation/gz/models/x500/model.sdf',
       'Tools/simulation/gz/models/x500_depth/model.sdf',
       'Tools/simulation/gz/models/x500_base/model.sdf',
       'ROMFS/px4fmu_common/init.d-posix/rcS']
for name in files:
    path=root/name
    print('\nSOURCE',name)
    text=path.read_text()
    if name.endswith('rcS'):
        for line in text.splitlines():
            if any(word in line for word in ('PX4_PARAM','param set','uxrce','PX4_GZ')):
                print(line)
    elif name.endswith('x500_base/model.sdf'):
        import xml.etree.ElementTree as ET
        doc=ET.fromstring(text)
        print('sensors:',[(s.get('name'),s.get('type')) for s in doc.iter('sensor')])
        print('plugins:',[(s.get('name'),s.get('filename')) for s in doc.iter('plugin')])
        print('includes:',[s.findtext('uri') for s in doc.iter('include')])
    else:
        print(text)
PY
