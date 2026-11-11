"""Prepare measured records from preserved native runs; no simulator commands."""
import argparse
from hashlib import sha256
import json
from pathlib import Path


def artifact(path,role):
    return {'path':path.as_posix(),'sha256':sha256(path.read_bytes()).hexdigest(),'role':role}


def prepare(prior,output,runs=None):
    data=json.loads(prior.read_text())
    data['runs']=[r for r in data['runs'] if r['run_id']!='classical-course-status-v1']
    for run in runs or ('gb10-sky-beta-outbound-v1','gb10-sky-beta-outbound-net-v2','gb10-sky-beta-outbound-net-v3'):
        base=Path('artifacts')/run; result=json.loads((base/'result.json').read_text()); mission=result['mission_report']
        if mission['state']=='COMPLETE':raise ValueError('success needs independent course grading; do not infer it from supervisor')
        provenance=[]
        for relative,wanted in result['source_snapshot_sha256'].items():
            item=artifact(base/'source'/relative,'code')
            if item['sha256']!=wanted:raise ValueError('source snapshot mismatch: '+relative)
            provenance.append(item)
        provenance.extend(artifact(path,'settings') for path in sorted((base/'settings').iterdir()) if path.is_file())
        provenance.extend(artifact(base/name,'evidence') for name in ('result.json','course-mission.json'))
        arm=next(e for e in mission['events'] if e['kind']=='armed_observed')
        record={'schema_version':1,'run_id':run,'strategy':'classical','kind':'course_flight','status':'failed',
            'reason':mission.get('abort_reason') or mission['reason'],'flight_approved':True,
            'approval_reference':mission['approval_reference'],'route':'window','clock_domain':'simulation',
            'sim_start_s':arm['armed_status_source_sim_s'],'sim_end_s':mission['streams']['position']['stamp_s'],
            'timing_scope':'native arm to last position consumed by the controller; not course finish',
            'contacts':None,'contact_observation_note':'No complete native contact stream; do not assume zero contacts or compute penalties.',
            'checkpoint_events':[],'provenance':provenance,
            'videos':[artifact(base/'video/preview.mp4','video')],
            'appearance_scenario':'original feature-poor sheets' if run.endswith('outbound-v1') else 'explicit estimated net strand appearance',
            'landing_scope':'aborted flight landed/disarmed; not on-pad course completion' if mission['state']=='ABORT_LANDED' else 'no landing/disarm established'}
        data['runs'].append(record)
    base=Path('artifacts/openfly-gb10-bf16-v2'); result=json.loads((base/'result.json').read_text())
    source=artifact(base/'source/model_benchmark.py','code')
    if source['sha256']!=result['script_sha256']:raise ValueError('actual model runner source differs from snapshot')
    if result['status']!='offline_inference_completed_not_flown':raise ValueError('completed offline inference evidence required')
    data['runs'].append({'schema_version':1,'run_id':'openfly-gb10-bf16-v2','strategy':'openfly',
        'kind':'offline_inference','status':'success','reason':'Recorded inputs, BF16 native GB10, no fine-tuning or flight commands; not Jetson-qualified.',
        'inference_latency_ms':[1000*s['wall_s'] for s in result['samples'] if not s['warmup']],
        'provenance':[source,artifact(base/'source/observations-course-rig-v1.json','settings'),
            artifact(base/'source/gb10-openfly-verified-v1.json','model'),artifact(base/'result.json','evidence')]})
    with output.open('x',encoding='utf-8') as stream:json.dump(data,stream,indent=2)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prior',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--runs',nargs='+')
    args=parser.parse_args();prepare(args.prior,args.output,args.runs)
