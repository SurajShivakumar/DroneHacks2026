"""Stop only orphan groups with open logs inside the named disarmed v21 run.

One-off recovery of an interrupted SSH diagnostic, not a flight/reset API.
Inspect the recorded targets. Never use this against an armed mission.
"""
import json,os,signal,time
from pathlib import Path

root=Path.home()/'dronehacks-2026-gb10'
run=(root/'artifacts/gb10-sky-beta-vio-quality-v21').resolve()
publication=json.loads((run/'vision-publication.json').read_text())
mission=json.loads((run/'course-mission.json').read_text())
if publication['arming_state']!=1 or mission.get('control_publishers_created') is not False:
    raise RuntimeError('not a confirmed disarmed/read-only diagnostic')
targets=[]
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():continue
    pid=int(entry.name)
    try:
        descriptors=[os.readlink(entry/'fd'/str(i)) for i in (1,2)]
        if any(str(run)+'/' in descriptor for descriptor in descriptors):
            group=os.getpgid(pid)
            if group!=os.getpgrp():targets.append(dict(pid=pid,group=group,logs=descriptors))
    except (FileNotFoundError,PermissionError,ProcessLookupError):continue
(run/'interrupted-cleanup.json').write_text(json.dumps(dict(reason='SSH disconnect; harness and observer gone without final report',targets=targets),indent=2))
groups=set(row['group'] for row in targets)
for group in groups:
    try:os.killpg(group,signal.SIGINT)
    except ProcessLookupError:pass
time.sleep(3)
for group in groups:
    try:os.killpg(group,signal.SIGKILL)
    except ProcessLookupError:pass
print(json.dumps(dict(stopped_groups=sorted(groups))))
