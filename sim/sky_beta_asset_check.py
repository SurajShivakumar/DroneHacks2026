"""Bounded passive Gazebo import/drop check; no PX4 or motor commands.

Scene pose is independent diagnostic evidence only and is never an input to
flight code, localization or policy inference.
"""
import argparse
from hashlib import sha256
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET


def run(project, model, output):
    output.mkdir(parents=True, exist_ok=False)
    tree=ET.parse(project/'artifacts/course-v3-r6/course.sdf')
    world=tree.find('world')
    include=ET.SubElement(world,'include')
    ET.SubElement(include,'uri').text='model://'+model.name
    ET.SubElement(include,'name').text='sky_beta_cad'
    ET.SubElement(include,'pose').text='-2.5146 .9144 .3 0 0 1.5707963267948966'
    scene=output/'world.sdf'; tree.write(scene,encoding='utf-8',xml_declaration=True)
    environment=dict(os.environ,GZ_PARTITION='sky_beta_asset_'+str(os.getpid()),
        GZ_SIM_RESOURCE_PATH=str(model.parent)+':'+str(project/'artifacts/course-v3-r6/models'))
    result={'kind':'passive_sky_beta_gazebo_import_not_flight','px4_started':False,
            'flight_commands_sent':False,'flight_model_ready':False,
            'model_sha256':sha256((model/'model.sdf').read_bytes()).hexdigest(),
            'world_sha256':sha256(scene.read_bytes()).hexdigest()}
    with (output/'gazebo.log').open('wb') as log:
        child=subprocess.Popen(['gz','sim','-r','-s','--headless-rendering','-v','3',str(scene)],
                               env=environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            for _ in range(15):
                time.sleep(1)
                if child.poll() is not None:
                    raise RuntimeError('Gazebo exited during asset import')
                listing=subprocess.run(['gz','topic','-l'],env=environment,capture_output=True,text=True,timeout=4)
                if '/world/dronehacks/dynamic_pose/info' in listing.stdout:
                    break
            else:
                raise RuntimeError('scene pose topic not discovered')
            time.sleep(3)
            pose=subprocess.run(['gz','topic','-e','-n','1','--json-output','-t','/world/dronehacks/dynamic_pose/info'],
                                env=environment,capture_output=True,text=True,timeout=8)
            (output/'diagnostic-poses.json').write_text(pose.stdout)
            if pose.returncode or not pose.stdout.strip():
                raise RuntimeError('no actual scene pose evidence')
            pose_data=json.loads(pose.stdout)
            actual=[p for p in pose_data.get('pose',[]) if p.get('name')=='sky_beta_cad']
            if len(actual)!=1 or not .03 < actual[0].get('position',{}).get('z',-1) < .12:
                raise RuntimeError('actual Sky Beta pose missing or not supported near floor')
            result['scene_pose_observed']=True
            result['actual_sky_beta_pose_observed']=actual[0]
        except Exception as error:
            result['error']=str(error)
            raise
        finally:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
            try:
                child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL); child.wait()
            result.update(child_exit_code=child.returncode,all_children_stopped=child.poll() is not None)
            log.flush()
            text=(output/'gazebo.log').read_text(errors='replace')
            result['contact_solver_warning']='contact hash table bucket overflow' in text.replace('contach','contact')
            if result['contact_solver_warning']:
                result['error']='contact solver overflow; model import alone does not qualify contacts'
            (output/'result.json').write_text(json.dumps(result,indent=2))
    if result.get('error'):
        raise RuntimeError(result['error'])
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    run(Path(__file__).resolve().parents[1],args.model.resolve(),args.output.resolve())
