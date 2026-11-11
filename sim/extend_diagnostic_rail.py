"""Make an exclusive disarmed diagnostic world with a longer fixture rail."""
import argparse
import json
from hashlib import sha256
from pathlib import Path
import xml.etree.ElementTree as ET


def extend(source,output,travel_m,velocity_mps=.4,height_m=None):
    if not .15<travel_m<=1.0:
        raise ValueError('diagnostic rail travel must be greater than 0.15 m and at most 1 m')
    if not .1<=velocity_mps<=1.5:
        raise ValueError('diagnostic rail speed must be 0.1..1.5 m/s')
    if height_m is not None and not .15<=height_m<=1.2:
        raise ValueError('diagnostic fixture height must be 0.15..1.2 m')
    tree=ET.parse(source)
    model=tree.find("world/model[@name='bootstrap_fixture']")
    joint=model.find("joint[@name='diagnostic_slide']") if model is not None else None
    axis=joint.find('axis') if joint is not None else None
    limit=axis.find('limit') if axis is not None else None
    if limit is None or float(limit.findtext('lower'))>-.15 or float(limit.findtext('upper'))<.15:
        raise ValueError('reviewed bootstrap fixture rail limits not found')
    # The fixture root inherits yaw=pi/2, so local +X moves world +Y,
    # through the corridor. Reject unknown geometry instead of assuming.
    root_pose=model.findtext('pose', default='').split()
    if len(root_pose) < 6 or abs(float(root_pose[5])-1.57079632679) > 0.02:
        raise ValueError('fixture root yaw is not pi/2; re-derive the corridor axis')
    if height_m is not None:
        detach=next((p for p in model.findall('plugin') if p.findtext('child_model')),None)
        aircraft=tree.find(f"world/model[@name='{detach.findtext('child_model')}']") if detach is not None else None
        if aircraft is None:
            raise ValueError('fixture aircraft must be present before changing physical height')
        for target in (model,aircraft):
            pose=target.find('pose'); values=pose.text.split()
            values[2]=str(height_m); pose.text=' '.join(values)
    axis.find('xyz').text='1 0 0'
    limit.find('lower').text=f'{-travel_m:.6f}'
    limit.find('upper').text=f'{travel_m:.6f}'
    limit.find('velocity').text=f'{velocity_mps:.6f}'
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('xb') as stream:tree.write(stream,encoding='utf-8',xml_declaration=True)
    report={'kind':'disarmed_vio_diagnostic_world_only','source_sha256':sha256(source.read_bytes()).hexdigest(),
        'world_sha256':sha256(output.read_bytes()).hexdigest(),'rail_travel_m':travel_m,'joint_axis_local':[1,0,0],
        'fixture_root_yaw_rad':float(root_pose[5]),'rail_axis_world':'course-forward +Y',
        'rail_velocity_limit_mps':velocity_mps,
        'physical_fixture_height_m':height_m,
        'flight_commands':False,'simulator_truth_to_navigation':False}
    output.with_suffix('.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--travel-m',type=float,default=.8)
    parser.add_argument('--velocity-mps',type=float,default=.4)
    parser.add_argument('--height-m',type=float)
    args=parser.parse_args();print(json.dumps(extend(args.source,args.output,args.travel_m,args.velocity_mps,args.height_m),indent=2))
