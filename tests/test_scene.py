from pathlib import Path
import tempfile
import struct
import unittest
import xml.etree.ElementTree as ET

from tools.build_course import build


class SceneTests(unittest.TestCase):
    def test_packaged_mesh_and_metadata_match_published_inputs(self):
        source = Path(__file__).resolve().parents[1] / "dronehacks-course-v3-obj"
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "scene"
            manifest = build(source, output)
            model = output / "models" / "dronehacks_course"
            self.assertFalse(manifest["runtime_validated"])
            self.assertEqual(len(manifest["geometry"]["groups"]), 33)
            for suffix in ("obj", "mtl"):
                name = f"dronehacks-course-v3.{suffix}"
                self.assertEqual((model / "meshes" / name).read_bytes(),
                                 (source / name).read_bytes())
            self.assertEqual(ET.parse(model / "model.sdf").findtext("model/static"), "true")
            self.assertEqual(len(ET.parse(model / "model.sdf").findall("model/link/visual")), 33)
            net_pieces = [piece for piece in manifest["visual_pieces"] if piece["net"]]
            self.assertEqual(len(net_pieces), 5)
            triangles = 0
            for piece in manifest["visual_pieces"]:
                raw = (model / "meshes" / piece["file"]).read_bytes()
                count = struct.unpack_from('<I', raw, 80)[0]
                self.assertEqual(len(raw), 84 + 50 * count)
                triangles += count
            self.assertEqual(triangles, 81484)
            self.assertEqual(sum(v.findtext("cast_shadows") == "false"
                                 for v in ET.parse(model / "model.sdf").findall("model/link/visual")), 5)
            world = ET.parse(output / "course.sdf")
            self.assertEqual(world.findtext("world/include/uri"), "model://dronehacks_course")
            classic = ET.parse(output / "course-classic.world")
            self.assertEqual(classic.getroot().get("version"), "1.6")
            self.assertEqual(classic.find("world/physics").get("type"), "ode")
            self.assertEqual(classic.findall("world/plugin"), [])

    def test_existing_scene_is_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(FileExistsError):
                build("not-needed", temporary)


if __name__ == "__main__":
    unittest.main()
