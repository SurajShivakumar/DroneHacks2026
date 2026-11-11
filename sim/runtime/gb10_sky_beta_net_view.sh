#!/usr/bin/env bash
# Sensor view at 1 m, physically held by a joint. No PX4, arming or flight.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd "$HOME/dronehacks-2026-gb10"
python3 - <<'PY'
from pathlib import Path
import xml.etree.ElementTree as ET
p=Path('artifacts/dronehacks_course_net_grid-v1')
tree=ET.parse(p/'world.sdf')
for name in ('sky_beta_estimated','bootstrap_fixture'):
 model=tree.find(f"world/model[@name='{name}']")
 if model is None:raise ValueError('missing reviewed model '+name)
 pose=model.find('pose'); fields=pose.text.split();fields[2]='1.0';pose.text=' '.join(fields)
with (p/'net-view-world.sdf').open('xb') as output:
 tree.write(output,encoding='utf-8',xml_declaration=True)
PY
python3 -m sim.gazebo_smoke --scene artifacts/dronehacks_course_net_grid-v1/net-view-world.sdf \
 --airframe-contract artifacts/sky-beta-estimated-v4/airframe-contract.json \
 --px4-model-name sky_beta_estimated --hardware-rendering --record-image \
 --output artifacts/gb10-sky-beta-net-view-v1
