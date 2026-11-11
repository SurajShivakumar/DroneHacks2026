"""Offline simulator truth extraction. Never imported by flight algorithms.

The raw Gazebo transport capture remains separate from ROS/VIO/controller data.
This records poses only; it does not establish contact-free course completion.
"""
import argparse
import csv
from hashlib import sha256
import json
from math import isfinite, sqrt
from pathlib import Path


def json_messages(stream, chunk_size=65536):
    decoder=json.JSONDecoder(); buffer=''
    while True:
        chunk=stream.read(chunk_size)
        buffer+=chunk
        while buffer.strip():
            buffer=buffer.lstrip()
            try:
                value,end=decoder.raw_decode(buffer)
            except json.JSONDecodeError:
                if not chunk:
                    raise ValueError('incomplete or invalid raw scene JSON capture')
                break
            yield value
            buffer=buffer[end:]
        if not chunk:
            return
        if len(buffer)>8*1024*1024:
            raise ValueError('unbounded or invalid scene message')


def model_samples(messages, model_name):
    last=None
    for message in messages:
        stamp=message['header']['stamp']
        seconds,nanoseconds=int(stamp.get('sec',0)),int(stamp.get('nsec',0))
        if seconds<0 or not 0<=nanoseconds<1_000_000_000:
            raise ValueError('invalid scene timestamp')
        ns=seconds*1_000_000_000+nanoseconds
        poses=[pose for pose in message.get('pose',[]) if pose.get('name')==model_name]
        if len(poses)!=1:
            raise ValueError('actual model pose missing or ambiguous')
        pose=poses[0]; position=pose['position']; orientation=pose['orientation']
        values=[float(position.get(axis,0)) for axis in ('x','y','z')]
        quaternion=[float(orientation.get(axis,0)) for axis in ('w','x','y','z')]
        if not all(isfinite(v) for v in values+quaternion) or abs(sqrt(sum(v*v for v in quaternion))-1)>.001:
            raise ValueError('invalid physical scene pose')
        if last is not None and ns<last:
            raise ValueError('scene clock regressed')
        if last==ns:
            continue
        last=ns
        yield [ns/1e9,*values,*quaternion]


def extract(raw, model, output):
    output.mkdir(parents=True,exist_ok=False)
    result={'kind':'independent_simulator_scene_poses_not_navigation_input',
        'model':model,'raw_sha256':sha256(raw.read_bytes()).hexdigest(),
        'course_completion_evaluated':False,'contact_evaluated':False}
    count=0; first=last=None; max_gap=0.
    try:
        with raw.open(encoding='utf-8') as stream,(output/'poses.csv').open('x',newline='',encoding='utf-8') as target:
            writer=csv.writer(target); writer.writerow(['t','x','y','z','qw','qx','qy','qz'])
            for row in model_samples(json_messages(stream),model):
                if last is not None: max_gap=max(max_gap,row[0]-last)
                else: first=row[0]
                last=row[0]; count+=1; writer.writerow(row)
        if count<2:
            raise ValueError('insufficient advancing scene evidence')
    except Exception as error:
        result['error']=str(error)
        raise
    finally:
        result.update(samples=count,first_source_sim_s=first,last_source_sim_s=last,
                      max_source_gap_s=max_gap)
        (output/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw',type=Path,required=True)
    parser.add_argument('--model',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(extract(args.raw,args.model,args.output),indent=2))
