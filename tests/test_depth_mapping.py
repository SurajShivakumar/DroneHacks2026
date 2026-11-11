import unittest

import numpy as np

from dronebench.depth_mapping import DepthMap, Pinhole


class DepthMappingTests(unittest.TestCase):
    def test_measured_ray_free_endpoint_occupied_behind_unknown(self):
        mapping = DepthMap((3, 3, 6), 1.)
        transform = np.eye(4)
        transform[:3, 3] = (.5, .5, .5)
        mapping.integrate(np.array([[3.]]), Pinhole(1, 1, 1, 1, 0, 0), transform)
        self.assertEqual(list(mapping.cells[0, 0]), [0, 0, 0, 100, -1, -1])
        self.assertTrue(np.all(mapping.cells[1:, :, :] == -1))

    def test_invalid_depth_does_not_clear_unknown(self):
        mapping = DepthMap((5, 5, 5), 1.)
        result = mapping.integrate(np.array([[np.nan, np.inf, 0., -1., 9.]]),
                                   Pinhole(5, 1, 1, 1, 0, 0), np.eye(4))
        self.assertEqual(result, {"valid_returns": 0, "invalid_pixels": 5})
        self.assertTrue(np.all(mapping.cells == -1))

    def test_optical_pinhole_projection_and_transform(self):
        mapping = DepthMap((6, 6, 6), 1.)
        transform = np.array([[0., 0., 1., .5], [-1., 0., 0., 3.5],
                              [0., -1., 0., 3.5], [0., 0., 0., 1.]])
        mapping.integrate(np.array([[2., 2.]]), Pinhole(2, 1, 2, 2, 0, 0), transform)
        self.assertEqual(mapping.cells[2, 3, 3], 100)
        self.assertEqual(mapping.cells[2, 2, 3], 100)

    def test_occupied_precedence_across_rays_and_frames(self):
        mapping = DepthMap((2, 2, 6), 1.)
        transform = np.eye(4)
        transform[:3, 3] = (.5, .5, .5)
        camera = Pinhole(2, 1, 1000, 1000, 0, 0)
        mapping.integrate(np.array([[2., 4.]]), camera, transform)
        self.assertEqual(mapping.cells[0, 0, 2], 100)
        mapping.integrate(np.array([[4., 4.]]), camera, transform)
        self.assertEqual(mapping.cells[0, 0, 2], 100)

    def test_ray_clipped_from_outside_and_exit_not_obstacle(self):
        mapping = DepthMap((2, 2, 3), 1.)
        transform = np.eye(4)
        transform[:3, 3] = (.5, .5, -1.)
        mapping.integrate(np.array([[5.]]), Pinhole(1, 1, 1, 1, 0, 0), transform)
        self.assertEqual(list(mapping.cells[0, 0]), [0, 0, 0])
        self.assertFalse(np.any(mapping.cells == 100))

    def test_height_envelope_requires_all_heights_observed(self):
        mapping = DepthMap((2, 2, 3), 1., origin=(-1., -1., 0.))
        mapping.cells[0, 0, :2] = 0
        mapping.cells[1, 0, :2] = (0, 100)
        mapping.cells[0, 1, 0] = 0
        grid = mapping.occupancy_slice(0., 2.)
        self.assertEqual(grid.cells, (0, 100, -1, -1))
        self.assertEqual((grid.origin_x, grid.origin_y), (-1., -1.))
        with self.assertRaises(ValueError):
            mapping.occupancy_slice(-.1, 2.)

    def test_reject_reflection_and_integer_depth_units(self):
        mapping = DepthMap((2, 2, 3), 1.)
        camera = Pinhole(1, 1, 1, 1, 0, 0)
        with self.assertRaises(ValueError):
            mapping.integrate(np.array([[2000]], dtype=np.uint16), camera, np.eye(4))
        reflection = np.diag([-1., 1., 1., 1.])
        with self.assertRaises(ValueError):
            mapping.integrate(np.array([[2.]]), camera, reflection)
