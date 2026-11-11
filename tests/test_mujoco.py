"""Optional scene integration checks; no controller or flight is run."""

from pathlib import Path
import unittest

try:
    import mujoco
    import numpy as np
except ImportError:
    mujoco = None

SCENE = Path(__file__).resolve().parents[1] / "artifacts/mujoco-course-v1/course.xml"


@unittest.skipUnless(mujoco is not None and SCENE.exists(), "build optional MuJoCo scene first")
class MuJoCoSceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = mujoco.MjModel.from_xml_path(str(SCENE))

    def setUp(self):
        self.data = mujoco.MjData(self.model)

    def place(self, position):
        self.data.qpos[:3] = position
        self.data.qpos[3:7] = [1, 0, 0, 0]
        mujoco.mj_forward(self.model, self.data)

    def contacts_with(self, group_key):
        result = []
        for contact in self.data.contact:
            names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or ""
                     for g in contact.geom]
            if any(group_key in name for name in names):
                result.append(contact)
        return result

    def test_tunnel_remains_open_and_shell_collides(self):
        self.place([2.3114, .4572, .9398])
        self.assertEqual(self.contacts_with("Tube_oval"), [])
        self.place([1.59, .4572, .9398])
        self.assertGreater(len(self.contacts_with("Tube_oval")), 0)

    def test_window_remains_open_and_sill_collides(self):
        self.place([-.47093936, -2.27189951, .762])
        self.assertEqual(self.contacts_with("C2a_chicane_wall"), [])
        self.place([-.47093936, -2.27189951, .2])
        self.assertGreater(len(self.contacts_with("C2a_chicane_wall")), 0)

    def test_motor_units_and_gravity_balance(self):
        self.place([-2.5146, .9144, .8])
        self.data.ctrl[:] = .8 * 9.80665 / 4
        mujoco.mj_forward(self.model, self.data)
        np.testing.assert_allclose(self.data.qacc[:6], 0, atol=1e-7)

    def test_cameras_and_no_position_fix(self):
        self.assertEqual(self.model.nu, 4)
        self.assertEqual(self.model.ncam, 3)
        self.assertEqual(self.model.nsensor, 3)
        expected = {int(mujoco.mjtSensor.mjSENS_GYRO), int(mujoco.mjtSensor.mjSENS_ACCELEROMETER),
                    int(mujoco.mjtSensor.mjSENS_RANGEFINDER)}
        self.assertEqual(set(self.model.sensor_type), expected)
