#!/usr/bin/env bash
# Display-only isolation: retain net collisions but omit net visuals in this diagnostic.
set -eo pipefail
source /opt/ros/humble/setup.bash
python3 - <<'PY'
from pathlib import Path
import json
import xml.etree.ElementTree as ET
from sim.gazebo_proxy import value,camera
base=Path('artifacts/course-v3-r5/models/dronehacks_course').resolve()
r=ET.parse('artifacts/gazebo-proxy-v7/world.sdf')
w=r.find('world')
for include in list(w.findall('include')):
    w.remove(include)
c=ET.parse(base/'model.sdf').find('model')
c.set('name','inline_course_net_diagnostic_only')
metadata=json.loads(Path('artifacts/course-v3-r5/scene_manifest.json').read_text())
net_names={f'piece_{i:02d}' for i,piece in enumerate(metadata['visual_pieces']) if piece['net']}
for link in c.findall('link'):
    for visual in list(link.findall('visual')):
        if visual.get('name') in net_names:
            link.remove(visual)
for uri in c.iter('uri'):
    uri.text=str(base/'meshes'/uri.text.rsplit('/',1)[-1])
w.append(c)
m=ET.SubElement(w,'model',name='observer_display_only')
value(m,'static','true'); value(m,'pose','0 0 7.5 0 1.57079632679 0')
l=ET.SubElement(m,'link',name='observer_link'); camera(l,'observer',0)
l.find('sensor/pose').text='0 0 0 0 0 0'
with Path('artifacts/net-diagnostic-world.sdf').open('xb') as stream:
    r.write(stream,encoding='utf-8',xml_declaration=True)
PY
python3 -m sim.gazebo_smoke --scene "$PWD/artifacts/net-diagnostic-world.sdf" \
    --output "$PWD/artifacts/net-diagnostic-v1"
