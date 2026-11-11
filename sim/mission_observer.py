"""Add a fixed display/recording camera, never a navigation input."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
from sim.gazebo_proxy import camera, value


def build(source, output):
    if output.exists():
        raise FileExistsError('preserve previous recording scenes')
    tree=ET.parse(source); world=tree.find('world')
    model=ET.SubElement(world,'model',name='recording_observer')
    # Square overhead view keeps the launch lane visible behind the tall wall.
    # The prior low oblique view hid the entire first hover; preserve its runs.
    value(model,'static','true'); value(model,'pose','0 0 8 0 1.5707963267948966 1.5707963267948966')
    link=ET.SubElement(model,'link',name='camera')
    camera(link,'observer',0)
    sensor=link.find('sensor'); sensor.find('pose').text='0 0 0 0 0 0'
    sensor.find('camera/horizontal_fov').text='.95'
    sensor.find('camera/image/width').text='960'; sensor.find('camera/image/height').text='960'
    output.parent.mkdir(parents=True,exist_ok=True)
    tree.write(output,encoding='utf-8',xml_declaration=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();build(args.source,args.output)
