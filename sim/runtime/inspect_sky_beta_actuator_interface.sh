#!/usr/bin/env bash
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
r=Path.home()/'dronehacks-runtime/PX4-Autopilot'
for pattern in ['src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp','src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessRotors.cpp','ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500']:
 p=r/pattern
 print('\nFILE',pattern)
 if p.exists():
  lines=p.read_text().splitlines()
  for i,line in enumerate(lines):
   if any(t in line for t in ['1000','_outputs','KM','moment_ratio','torque =','thrust =','PX4_SIM_MODEL','CA_ROTOR','GZ_EC','THR_MDL','PWM_MAIN']):
    print(i+1, '\n'.join(lines[max(0,i-1):i+3]))
PY
