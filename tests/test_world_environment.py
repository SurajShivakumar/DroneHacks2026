import xml.etree.ElementTree as ET

from sim.world_environment import build


def test_environment_does_not_add_navigation_sensor(tmp_path):
    source, reference, output = (tmp_path / name for name in ('source.sdf', 'ref.sdf', 'out.sdf'))
    source.write_text('<sdf><world><model name="drone"><link name="base"/></model></world></sdf>')
    reference.write_text('<sdf><world><magnetic_field>1 2 3</magnetic_field><spherical_coordinates><latitude_deg>47</latitude_deg></spherical_coordinates></world></sdf>')
    original = source.read_bytes()
    build(source, reference, output)
    doc = ET.parse(output)
    assert doc.findtext('world/magnetic_field') == '1 2 3'
    assert doc.findtext('world/spherical_coordinates/latitude_deg') == '47'
    assert doc.find('.//sensor') is None
    assert doc.find('.//plugin') is None
    assert source.read_bytes() == original
