#!/usr/bin/env bash
# Post-flight: export independent truth poses and encode source-timed videos (no flight).
set -e
cd ~/dronehacks-2026-gb10
for n in "$@"; do
d=artifacts/gb10-sky-beta-course-v$n
python3 - "$d" <<'PY'
import json,sys
d=sys.argv[1]; rows=[]
for l in open(d+'/scene-poses.log'):
    try: r=json.loads(l)
    except Exception: continue
    h=r['header']['stamp']; t=float(h.get('sec',0))+float(h.get('nsec',0))/1e9
    p=[q for q in r['pose'] if q['name']=='sky_beta_estimated'][0]; o=p['orientation']; ps=p['position']
    rows.append((t,ps.get('x',0.),ps.get('y',0.),ps.get('z',0.),o.get('w',0.),o.get('x',0.),o.get('y',0.),o.get('z',0.)))
rows.sort(); out=[rows[0]]
for r in rows[1:]:
    if r[0]>out[-1][0]: out.append(r)
with open(d+'/truth-poses.csv','w') as f:
    f.write('t,x,y,z,vx,vy,vz,qw,qx,qy,qz\n')
    for r in out: f.write(','.join(map(repr,[r[0],r[1],r[2],r[3],0.,0.,0.,r[4],r[5],r[6],r[7]]))+'\n')
m=json.load(open(d+'/course-mission.json')); e={x['kind']:x for x in m['events']}
a=e['armed_observed']['sim_s']; b=e['landed_disarmed']['disarmed_source_sim_s']
open(d+'/video-window.txt','w').write(f'{a-2:.3f} {b+2:.3f} {b-a:.2f}\n')
print(d,'truth',len(out),'window',a-2,b+2)
PY
read S E T < $d/video-window.txt
rm -rf $d/flight-video
python3 tools/encode_flight_video.py --observer $d/rgb-sequence --onboard $d/image-sequence-4 --start-s $S --end-s $E \
  --output $d/flight-video --title "Sky Beta v$n: full course 9/9 gates, pad landing, ${T} s arm-to-disarm" > $d/encode.log 2>&1
ls -la $d/flight-video | head
done
echo POST_DONE
