from hashlib import sha256
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from sim.gazebo_proxy import build


class ProxyTests(unittest.TestCase):
    def test_sensor_setup_preserves_upstream_and_has_no_gnss_or_fake_vision(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            models = root / "models"
            for name in ("x500", "x500_base"):
                (models / name).mkdir(parents=True)
            base = models / "x500_base/model.sdf"
            base.write_text('<sdf><model name="x500_base"><pose>0 0 .24 0 0 0</pose><link name="base_link">'
                            '<sensor name="navsat" type="navsat"/>'
                            '<sensor name="imu" type="imu"/></link></model></sdf>')
            motors = models / "x500/model.sdf"
            motors.write_text('<sdf><model><plugin name="motor" filename="motor-library"/></model></sdf>')
            course = root / "course.sdf"
            course.write_text('<sdf version="1.9"><world name="dronehacks"/></sdf>')
            original_hash = sha256(base.read_bytes()).hexdigest()
            metadata = build(models, course, root / "output")
            self.assertEqual(sha256(base.read_bytes()).hexdigest(), original_hash)
            doc = ET.parse(root / "output/world.sdf")
            poses = doc.findall("world/model/pose")
            self.assertEqual(len(poses), 1, "upstream pose must be replaced, not duplicated")
            self.assertAlmostEqual(float(poses[0].text.split()[0]), -2.5146)
            sensors = list(doc.iter("sensor"))
            self.assertFalse(any(s.get("type") == "navsat" for s in sensors))
            self.assertEqual(sum(s.get("type") == "camera" for s in sensors), 3)
            self.assertEqual(sum(s.get("type") == "depth_camera" for s in sensors), 1)
            self.assertTrue(any(p.get("filename") == "motor-library" for p in doc.iter("plugin")))
            self.assertFalse(metadata["position_truth_publisher"])
