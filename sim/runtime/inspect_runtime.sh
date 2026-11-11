#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$HOME/dronehacks-runtime/ros_ws/install/setup.bash"
python3 - <<'PY'
from pathlib import Path
from hashlib import sha256
import json, subprocess
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition, VehicleLandDetected
runtime=Path.home()/'dronehacks-runtime'
binary=runtime/'PX4-Autopilot/build/px4_sitl_default/bin/px4'
record={'kind':'local_proxy_runtime_not_organizer_configuration',
        'sources':json.loads((runtime/'source-lock.json').read_text()),
        'gazebo_version':subprocess.check_output(['gz','sim','--versions'],text=True).strip(),
        'packages':subprocess.check_output(['dpkg-query','-W','-f=${Package}=${Version}\n',
                    'ros-humble-ros-base','ros-humble-ros-gzharmonic','gz-harmonic','libgz-sim8'],text=True).splitlines(),
        'px4_binary_sha256':sha256(binary.read_bytes()).hexdigest(),
        'px4_schemas':{cls.__name__: {'fields':list(cls.get_fields_and_field_types()),
                                    'message_version':getattr(cls,'MESSAGE_VERSION',None)}
                       for cls in (VehicleStatus,VehicleLocalPosition,VehicleLandDetected)}}
path=Path('artifacts/runtime/runtime-lock.json')
with path.open('x') as stream:
    json.dump(record,stream,indent=2)
print(json.dumps(record,indent=2))
PY
