import xml.etree.ElementTree as ET
import pytest
from sim.extend_diagnostic_rail import extend


def test_height_moves_fixture_and_aircraft_together_without_sensor_edits(tmp_path):
    source=tmp_path/'base.sdf'
    source.write_text('''<sdf><world><model name="sky"><pose>0 0 .15 0 0 1.57079632679</pose>
    <link name="base_link"><sensor name="stereo"><pose>.1 0 0 0 0 0</pose></sensor></link></model>
    <model name="bootstrap_fixture"><pose>0 0 .15 0 0 1.57079632679</pose>
    <joint name="diagnostic_slide"><axis><xyz>1 0 0</xyz><limit>
    <lower>-.15</lower><upper>.15</upper><velocity>.4</velocity></limit></axis></joint>
    <plugin><child_model>sky</child_model></plugin></model></world></sdf>''')
    out=tmp_path/'raised.sdf'
    extend(source,out,.8,1.,.9)
    tree=ET.parse(out)
    for name in ('sky','bootstrap_fixture'):
        assert tree.findtext(f"world/model[@name='{name}']/pose").split()[2]=='0.9'
    assert tree.findtext("world/model[@name='sky']/link/sensor/pose")=='.1 0 0 0 0 0'
    with pytest.raises(ValueError,match='height'):
        extend(source,tmp_path/'bad.sdf',.8,1.,2.)
