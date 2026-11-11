"""Copy pinned PX4 world environment metadata without adding a navigation fix."""
from copy import deepcopy
from pathlib import Path
import argparse
import xml.etree.ElementTree as ET


def build(source, reference, output):
    if output.exists():
        raise FileExistsError('Preserve earlier scenes')
    tree, upstream = ET.parse(source), ET.parse(reference)
    world, defaults = tree.find('world'), upstream.find('world')
    for name in ('spherical_coordinates', 'magnetic_field'):
        node = defaults.find(name)
        if node is None:
            raise ValueError('Pinned environmental metadata missing')
        existing = world.find(name)
        if existing is not None:
            world.remove(existing)
        world.append(deepcopy(node))
    # This is magnetometer/physics environment setup only: no GNSS sensor,
    # odometry publisher or PX4 parameter is added here.
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('xb') as stream:
        tree.write(stream, encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('reference', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    build(args.source, args.reference, args.output)
