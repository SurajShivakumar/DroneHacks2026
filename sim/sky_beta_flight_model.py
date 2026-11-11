"""Estimated Sky Beta propulsion/sensors on the actual CAD rigid-body model.

Simulation setup only. Parameters are explicit estimates authorized by the team,
not claimed hardware measurements. Generates matching physical PX4 allocation.
"""
import argparse
from copy import deepcopy
from hashlib import sha256
import json
from math import pi
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from sim.gazebo_proxy import camera, value
from sim.sky_beta_model import numbers
from tools.import_sky_beta_cad import aggregate_mass_properties


def box_inertia(mass, size):
    x,y,z=np.asarray(size)
    return np.diag([y*y+z*z,x*x+z*z,x*x+y*y])*mass/12


def normalized_thrust_for_force(force, coefficient, idle, maximum):
    """Invert native THR_MDL_FAC=1 then ESC idle..max speed scaling.

    This is the command estimate, not a claim that control allocation perfectly
    linearizes unequal rotor forces. Closed-loop validation remains required.
    """
    speed=np.sqrt(np.asarray(force)/coefficient)
    if not 0 < idle < maximum or np.any(speed < idle) or np.any(speed > maximum):
        raise ValueError('force outside spinning armed motor envelope')
    return ((speed-idle)/(maximum-idle))**2


def build(cad_dir, rigid_model, sensor_template, course_world, estimates_path, output):
    output.mkdir(parents=True,exist_ok=False)
    shutil.copytree(rigid_model/'meshes',output/'meshes')
    cad=json.loads((cad_dir/'cad-properties.json').read_text())
    estimates=json.loads(estimates_path.read_text())
    base_parts=[deepcopy(p) for p in cad['parts'] if p['mass_kg']>0]
    for component in estimates['additional_components']:
        base_parts.append({'name':component['name'],'mass_kg':component['mass_kg'],
            'com_flu_m':component['com_cad_flu_m'],
            'inertia_about_part_com_flu_kg_m2':box_inertia(component['mass_kg'],component['box_size_m']).tolist()})
    motor_parts=[p for p in base_parts if 'Motor' in p['name']]
    # Native quad-X numbering by actual physical FLU coordinates, NOT CAD names.
    ordered=[]
    for front,left in [(True,False),(False,True),(True,True),(False,False)]:
        matching=[p for p in motor_parts if (p['com_flu_m'][0]>0)==front and (p['com_flu_m'][1]>0)==left]
        if len(matching)!=1:
            raise ValueError('cannot establish unambiguous physical quad motor order')
        ordered.append(matching[0])
    prop=estimates['propeller']; radius=prop['diameter_m']/2; pmass=prop['mass_kg_each']
    prop_inertia=np.diag([pmass*(radius*radius/4+prop['thickness_m']**2/12)]*2+[pmass*radius*radius/2])
    prop_parts=[]
    for i,part in enumerate(ordered):
        center=[*part['com_flu_m'][:2],part['bounds_flu_m'][1][2]-prop['plane_below_shaft_tip_m']]
        prop_parts.append({'name':f'estimated propeller {i}','mass_kg':pmass,'com_flu_m':center,
                          'inertia_about_part_com_flu_kg_m2':prop_inertia.tolist()})
    mass,com,inertia=aggregate_mass_properties(base_parts+prop_parts)
    base_mass,base_com,base_inertia=aggregate_mass_properties(base_parts)
    old_com=np.asarray(cad['com_flu_m'])
    model_tree=ET.parse(rigid_model/'model.sdf'); model=model_tree.find('model')
    model.set('name','sky_beta_estimated')
    link=model.find("link[@name='base_link']")
    link.remove(link.find('inertial'))
    inertial=ET.SubElement(link,'inertial')
    value(inertial,'pose',numbers([*(base_com-com),0,0,0])); value(inertial,'mass',base_mass)
    tensor=ET.SubElement(inertial,'inertia')
    for tag,i,j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:
        value(tensor,tag,base_inertia[i,j])
    for element in list(link.findall('visual'))+list(link.findall('collision')):
        pose=list(map(float,element.findtext('pose').split()))
        pose[:3]=np.asarray(pose[:3])+old_com-com
        element.find('pose').text=numbers(pose)
        for uri in element.findall('geometry/mesh/uri'):
            uri.text='model://'+output.name+'/'+uri.text
    for index,component in enumerate(estimates['additional_components']):
        position=np.asarray(component['com_cad_flu_m'])-com
        for category in ('visual','collision'):
            element=ET.SubElement(link,category,name=f'extra_{category}_{index}')
            value(element,'pose',numbers([*position,0,0,0]))
            value(ET.SubElement(ET.SubElement(element,'geometry'),'box'),'size',numbers(component['box_size_m']))
    physical=estimates['motor']
    coefficient=physical['max_thrust_N_each']/physical['max_speed_rad_s']**2
    idle=physical['armed_idle_speed_rad_s']
    if not 0 < idle < physical['max_speed_rad_s']:
        raise ValueError('estimated spinning armed idle must be below maximum')
    params={'CA_AIRFRAME':0,'CA_ROTOR_COUNT':4,'THR_MDL_FAC':1.}
    allocation=[]
    for i,(part,direction) in enumerate(zip(prop_parts,physical['spin_pattern'])):
        center=np.asarray(part['com_flu_m'])-com
        rotor=ET.SubElement(model,'link',name=f'rotor_{i}')
        value(rotor,'pose',numbers([*center,0,0,0]))
        item=ET.SubElement(rotor,'inertial'); value(item,'mass',pmass)
        tensor=ET.SubElement(item,'inertia')
        for tag,axis in [('ixx',0),('iyy',1),('izz',2)]: value(tensor,tag,prop_inertia[axis,axis])
        for category in ('visual','collision'):
            element=ET.SubElement(rotor,category,name=f'prop_{i}_{category}')
            cylinder=ET.SubElement(ET.SubElement(element,'geometry'),'cylinder')
            value(cylinder,'radius',radius); value(cylinder,'length',prop['thickness_m'])
            if category=='visual':
                material=ET.SubElement(element,'material'); value(material,'ambient','.2 .2 .2 .3');value(material,'diffuse','.2 .2 .2 .3')
        joint=ET.SubElement(model,'joint',name=f'rotor_{i}_joint',type='revolute')
        value(joint,'parent','base_link');value(joint,'child',f'rotor_{i}')
        axis=ET.SubElement(joint,'axis');value(axis,'xyz','0 0 1')
        limit=ET.SubElement(axis,'limit'); value(limit,'lower',-1e16);value(limit,'upper',1e16)
        plugin=ET.SubElement(model,'plugin',filename='gz-sim-multicopter-motor-model-system',name='gz::sim::systems::MulticopterMotorModel')
        for key,val in {'jointName':f'rotor_{i}_joint','linkName':f'rotor_{i}','turningDirection':direction,
            'timeConstantUp':physical['rise_time_constant_s'],'timeConstantDown':physical['fall_time_constant_s'],
            'maxRotVelocity':physical['max_speed_rad_s'],'motorConstant':coefficient,
            'momentConstant':physical['reaction_torque_per_thrust_m'],'commandSubTopic':'command/motor_speed',
            'motorNumber':i,'rotorDragCoefficient':physical['rotor_drag_coefficient'],
            'rollingMomentCoefficient':physical['rolling_moment_coefficient'],'rotorVelocitySlowdownSim':10,'motorType':'velocity'}.items():
            value(plugin,key,val)
        frd=center*np.array([1.,-1.,-1.])
        for axis_name,number in zip(('PX','PY','PZ'),frd): params[f'CA_ROTOR{i}_{axis_name}']=float(number)
        km=physical['reaction_torque_per_thrust_m']*(1 if direction=='ccw' else -1)
        params[f'CA_ROTOR{i}_KM']=km
        params[f'SIM_GZ_EC_MIN{i+1}']=idle;params[f'SIM_GZ_EC_MAX{i+1}']=physical['max_speed_rad_s']
        params[f'SIM_GZ_EC_FUNC{i+1}']=101+i
        # FLU torque: r cross Fz plus opposite reaction torque of the prop spin.
        allocation.append([center[1],-center[0],-km,1.])
    hover_forces=np.linalg.solve(np.asarray(allocation).T,[0,0,0,mass*9.80665])
    if np.any(hover_forces<=0) or np.max(hover_forces)>physical['max_thrust_N_each']:
        raise ValueError('estimated physical rotor geometry cannot support balanced hover')
    params['MPC_THR_HOVER']=float(np.mean(normalized_thrust_for_force(
        hover_forces,coefficient,idle,physical['max_speed_rad_s'])))
    template=ET.parse(sensor_template).find("world/model[@name='dronehacks_x500_proxy']/link[@name='base_link']")
    imu_part=next(p for p in base_parts if p['name']=='Pixhawk 6C - Underside')
    imu_position=np.asarray(imu_part['com_flu_m'])-com
    for sensor in template.findall('sensor'):
        if sensor.get('type') in ('imu','magnetometer','air_pressure'):
            clone=deepcopy(sensor)
            if clone.find('pose') is None: value(clone,'pose','0 0 0 0 0 0')
            clone.find('pose').text=numbers([*imu_position,0,0,0])
            link.append(clone)
    for tag,number in zip(('X','Y','Z'),imu_position*np.array([1,-1,-1])):
        params['EKF2_IMU_POS_'+tag]=float(number)
    camera_part=next(p for p in base_parts if p['name']=='OAK-D2 Camera')
    camera_center=np.asarray([(camera_part['bounds_flu_m'][1][0]-.001),0.,
                             np.mean(np.asarray(camera_part['bounds_flu_m'])[:,2])])-com
    for name,side in [('left',.5),('right',-.5),('rgb',0),('ideal_depth',0)]:
        camera(link,name,0,'depth_camera' if name=='ideal_depth' else 'camera')
        sensor=link.findall('sensor')[-1]
        position=camera_center+np.array([0,side*estimates['camera']['stereo_baseline_m'],0])
        sensor.find('pose').text=numbers([*position,0,0,0])
    range_position=np.asarray(estimates['additional_components'][1]['com_cad_flu_m'])-com
    ranger=deepcopy(template.find("sensor[@name='down_range']"))
    ranger.find('pose').text=numbers([*range_position,0,pi/2,0]);link.append(ranger)
    body_from_imu=np.eye(4);body_from_imu[:3,3]=imu_position
    model_tree.write(output/'model.sdf',encoding='utf-8',xml_declaration=True)
    shutil.copy2(rigid_model/'model.config',output/'model.config')
    course=ET.parse(course_world); world=course.find('world')
    for name in ('Imu','AirPressure','Magnetometer'):
        slug={'Imu':'imu','AirPressure':'air-pressure','Magnetometer':'magnetometer'}[name]
        ET.SubElement(world,'plugin',filename=f'gz-sim-{slug}-system',name=f'gz::sim::systems::{name}')
    value(model,'pose','-2.5146 .9144 .15 0 0 1.5707963267948966');world.append(model)
    environment=ET.parse(sensor_template).find('world')
    for tag in ('spherical_coordinates','magnetic_field'):
        world.append(deepcopy(environment.find(tag)))
    course.write(output/'world.sdf',encoding='utf-8',xml_declaration=True)
    envelope=max(np.linalg.norm(np.asarray(p['com_flu_m'])[:2]-com[:2])+radius for p in prop_parts)
    contract={'kind':'estimated_sky_beta_cad_with_estimated_propulsion',
        'cad_properties_sha256':sha256((cad_dir/'cad-properties.json').read_bytes()).hexdigest(),
        'rigid_model_sha256':sha256((rigid_model/'model.sdf').read_bytes()).hexdigest(),
        'generator_sha256':sha256(Path(__file__).read_bytes()).hexdigest(),
        'world_sha256':sha256((output/'world.sdf').read_bytes()).hexdigest(),
        'mass_kg':mass,'cad_mass_kg':cad['cad_mass_kg_excluding_omissions'],'com_cad_flu_m':com.tolist(),
        'inertia_com_flu_kg_m2':inertia.tolist(),'base_mass_kg':base_mass,'propeller_mass_kg':4*pmass,
        'collision_radius_m':float(envelope),'collision_half_height_m':.13,
        'body_from_imu':body_from_imu.tolist(),'range_offset_frd_m':(range_position*np.array([1,-1,-1])).tolist(),
        'range_axis_frd':[0.,0.,1.],'px4_parameters':params,'balanced_hover_forces_N':hover_forces.tolist(),
        'motor_coefficient_N_per_rad_s_squared':coefficient,'max_thrust_N_each':physical['max_thrust_N_each'],
        'estimates_sha256':sha256(estimates_path.read_bytes()).hexdigest(),'assumptions':estimates,
        'missing':[],'estimated_parameters':True,'motor_plugins_enabled':True,'flight_model_ready':True,
        'hardware_qualified':False,'flight_model_ready_scope':'self-consistent estimated SITL model, not measured hardware dynamics'}
    (output/'airframe-contract.json').write_text(json.dumps(contract,indent=2))
    return contract


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('cad','rigid-model','sensor-template','course','estimates','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    result=build(args.cad,args.rigid_model,args.sensor_template,args.course,args.estimates,args.output)
    print(json.dumps({k:v for k,v in result.items() if k not in ('assumptions','px4_parameters')},indent=2))
