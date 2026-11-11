import unittest
from types import SimpleNamespace

import numpy as np

from dronebench.published_wall_map import build_published_wall_map


class PublishedWallMapTests(unittest.TestCase):
    @staticmethod
    def course(**kwargs):
        values = dict(box_c=np.array([[1., 2., 1.]]),
                      box_u=np.array([[1., 0.]]),
                      box_h=np.array([[2., .25, .5]]), box_names=["wall"])
        values.update(kwargs)
        return SimpleNamespace(**values)

    def test_rotated_box_uses_true_obb_edges(self):
        theta = .61
        u = np.array([np.cos(theta), np.sin(theta)])
        v = np.array([-u[1], u[0]])
        course = self.course(box_u=u[None])
        result = build_published_wall_map(course, z=1.)
        expected_corners = np.array([np.array([1., 2.]) + a * 2 * u + b * .25 * v
                                     for a, b in [(-1, -1), (1, -1), (1, 1), (-1, 1)]])
        np.testing.assert_allclose(result.segments[:, 0], expected_corners)
        np.testing.assert_allclose(result.segments[:, 1], np.roll(expected_corners, -1, axis=0))
        np.testing.assert_allclose(np.linalg.norm(np.diff(result.segments, axis=1)[:, 0], axis=1),
                                   [4., .5, 4., .5])
        self.assertEqual(result.labels, tuple(f"box:0:wall:edge:{i}" for i in range(4)))
        self.assertFalse(result.segments.flags.writeable)

    def test_height_slice_excludes_boxes_above_and_below(self):
        course = self.course(box_c=np.array([[0., 0., .5], [3., 0., 2.5]]),
                             box_u=np.array([[1., 0.], [0., 1.]]),
                             box_h=np.array([[1., .1, .5], [2., .2, .5]]),
                             box_names=["low", "high"])
        for z, prefix in ((0., "box:0:low:"), (1., "box:0:low:"),
                           (2., "box:1:high:"), (3., "box:1:high:")):
            result = build_published_wall_map(course, z=z)
            self.assertEqual(result.segments.shape, (4, 2, 2))
            self.assertTrue(all(label.startswith(prefix) for label in result.labels))
        result = build_published_wall_map(course, z=1.5)
        self.assertEqual(result.segments.shape, (0, 2, 2))
        self.assertEqual(result.labels, ())

    def test_only_explicit_net_bounds_are_added(self):
        course = self.course()
        self.assertEqual(build_published_wall_map(course, z=10.).segments.shape, (0, 2, 2))
        result = build_published_wall_map(course, z=10., net_bounds=(-4., 6., -2., 8.))
        corners = np.array([[-4., -2.], [6., -2.], [6., 8.], [-4., 8.]])
        np.testing.assert_array_equal(result.segments[:, 0], corners)
        np.testing.assert_array_equal(result.segments[:, 1], np.roll(corners, -1, axis=0))
        self.assertEqual(result.labels, ("explicit_net:ymin", "explicit_net:xmax",
                                         "explicit_net:ymax", "explicit_net:xmin"))
        self.assertEqual(result.source_scope, "published_course_boxes_and_explicit_net")

    def test_net_appended_without_hiding_box_source(self):
        result = build_published_wall_map(self.course(), z=1., net_bounds=(-4, 4, -5, 5))
        self.assertEqual(result.segments.shape, (8, 2, 2))
        self.assertIn("tube curved shell", result.omitted_geometry)
        self.assertIn("boulders", result.omitted_geometry)
        self.assertEqual(result.z, 1.)

    def test_non_box_geometry_is_not_invented(self):
        course = self.course(tube={"a": 20}, cyl_xy=np.array([[20, 30]]), cyl_r=np.array([5]))
        result = build_published_wall_map(course, z=1.)
        self.assertEqual(result.segments.shape, (4, 2, 2))
        self.assertEqual(result.source_scope, "published_course_boxes")

    def test_parser_empty_arrays_and_shaped_empty_arrays(self):
        for c, u, h in (([], [], []), (np.empty((0, 3)), np.empty((0, 2)), np.empty((0, 3)))):
            result = build_published_wall_map(self.course(box_c=c, box_u=u, box_h=h, box_names=[]), z=0.)
            self.assertEqual(result.segments.shape, (0, 2, 2))

    def test_invalid_box_shapes_extents_axes_and_names_rejected(self):
        for kwargs in (dict(box_c=[[0, 0]]), dict(box_u=[[1, 0], [1, 0]]),
                       dict(box_h=[[1, -.1, 1]]), dict(box_h=[[1, 0, 1]]),
                       dict(box_h=[[1, 1, np.nan]]), dict(box_c=[[np.inf, 0, 0]]),
                       dict(box_u=[[0, 0]]), dict(box_u=[[2, 0]]),
                       dict(box_u=[[np.nan, 0]]), dict(box_names=[])):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                build_published_wall_map(self.course(**kwargs), z=100.)

    def test_invalid_height_and_net_bounds_rejected(self):
        for z in (np.nan, np.inf, -np.inf):
            with self.assertRaises(ValueError):
                build_published_wall_map(self.course(), z=z)
        for bounds in ((0, 0, -1, 1), (2, -2, -1, 1), (-1, 1, 0, 0),
                       (-1, 1, np.nan, 2), (-1, 1, 2)):
            with self.assertRaises(ValueError):
                build_published_wall_map(self.course(), z=1., net_bounds=bounds)


if __name__ == "__main__":
    unittest.main()
