from math import pi
import unittest

from sim.vio_calibration import checked_intrinsics, optical_to_imu


class ExtrinsicTests(unittest.TestCase):
    def test_optical_axes_and_lever_arm(self):
        t = optical_to_imu([.13233, .0375, .26078, 0, 0, 0], [0, 0, 0, 0, 0, 0])
        self.assertEqual([row[2] for row in t[:3]], [1, 0, 0])  # optical forward = body forward
        self.assertEqual([row[0] for row in t[:3]], [0, -1, 0])  # optical right = body right
        self.assertEqual([row[3] for row in t[:3]], [.13233, .0375, .26078])

    def test_rotated_offset_imu_is_not_treated_as_body_origin(self):
        t = optical_to_imu([1, 2, 3, 0, 0, 0], [1, 1, 3, 0, 0, pi / 2])
        self.assertAlmostEqual(t[0][3], 1)
        self.assertAlmostEqual(t[1][3], 0)
        self.assertAlmostEqual(t[2][3], 0)
