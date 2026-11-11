#!/usr/bin/env bash
# Observer camera is an inspection aid only; never a navigation input.
set -eo pipefail
source /opt/ros/humble/setup.bash
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
from sim.gazebo_proxy import value, camera
r=ET.parse('artifacts/gazebo-proxy-v7/world.sdf')
w=r.find('world')
m=ET.SubElement(w,'model',name='observer_display_only')
value(m,'static','true'); value(m,'pose','0 0 7.5 0 1.57079632679 0')
l=ET.SubElement(m,'link',name='observer_link')
camera(l,'observer',0)
l.find('sensor/pose').text='0 0 0 0 0 0'
with Path('artifacts/overview-diagnostic-world.sdf').open('xb') as stream:
    r.write(stream,encoding='utf-8',xml_declaration=True)
PY
python3 -m sim.gazebo_smoke --scene "$PWD/artifacts/overview-diagnostic-world.sdf" \
    --output "$PWD/artifacts/overview-diagnostic-v1"
