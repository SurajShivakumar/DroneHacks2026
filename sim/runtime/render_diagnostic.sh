#!/usr/bin/env bash
# Primitive scene to distinguish rendering faults from course/airframe occlusion.
set -eo pipefail
source /opt/ros/humble/setup.bash
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
from sim.gazebo_proxy import camera, value
source=Path('artifacts/course-v3-r4/course.sdf')
r=ET.parse(source)
w=r.find('world')
for include in list(w.findall('include')):
    w.remove(include)
model=ET.SubElement(w,'model',name='render_diagnostic_only')
value(model,'static','true')
link=ET.SubElement(model,'link',name='sensor_rig')
value(link,'pose','0 0 .5 0 0 0')
for name,y in (('left',.0375),('right',-.0375),('rgb',0),('ideal_depth',0)):
    camera(link,name,y,'depth_camera' if name=='ideal_depth' else 'camera')
imu=ET.SubElement(link,'sensor',name='imu_sensor',type='imu')
value(imu,'update_rate',240)
ET.SubElement(w,'plugin',filename='gz-sim-imu-system',name='gz::sim::systems::Imu')
box=ET.SubElement(w,'model',name='known_red_cube')
value(box,'static','true'); value(box,'pose','2 0 .7 0 0 0')
visual=ET.SubElement(ET.SubElement(box,'link',name='cube'),'visual',name='red')
value(ET.SubElement(ET.SubElement(visual,'geometry'),'box'),'size','.7 .7 .7')
material=ET.SubElement(visual,'material')
value(material,'ambient','.2 .01 .01 1'); value(material,'diffuse','.8 .04 .04 1')
path=Path('artifacts/render-diagnostic-world.sdf')
with path.open('xb') as stream:
    r.write(stream,encoding='utf-8',xml_declaration=True)
PY
python3 -m sim.gazebo_smoke --scene "$PWD/artifacts/render-diagnostic-world.sdf" \
    --output "$PWD/artifacts/render-diagnostic-v1"
