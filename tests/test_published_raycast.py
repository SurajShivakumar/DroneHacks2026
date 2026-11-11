import unittest
from types import SimpleNamespace

import numpy as np

from dronebench.published_raycast import build_published_raycast, raycast_published_course


class PublishedRaycastTests(unittest.TestCase):
    @staticmethod
    def course(**kwargs):
        values = dict(box_c=[[3., 0., 1.]], box_u=[[1., 0.]],
                      box_h=[[1., .5, 1.]], box_names=["wall"])
        values.update(kwargs)
        return SimpleNamespace(**values)

    def build(self, course=None, **kwargs):
        return build_published_raycast(self.course() if course is None else course,
                                      published_provenance="analytic published fixture", **kwargs)

    def empty(self, **kwargs):
        return self.build(self.course(box_c=[], box_u=[], box_h=[], box_names=[]), **kwargs)

    def test_non_normalized_parameter_is_optical_depth(self):
        model = self.build(self.course(box_c=[[0., 0., 6.]], box_h=[[10., 10., 1.]]))
        hit = model.cast([0, 0, 0], [[.5, .2, 1], [1, .4, 2]])
        np.testing.assert_allclose(hit.t, [5., 2.5])
        np.testing.assert_allclose(hit.normals, [[0, 0, -1], [0, 0, -1]])
        self.assertEqual(hit.labels.tolist(), ["box:0:wall"] * 2)
        self.assertFalse(hit.origin_inside_solid)

    def test_rotated_box_analytic_entry_and_normal(self):
        theta = .61
        u = np.array([np.cos(theta), np.sin(theta), 0.])
        centre = np.array([3., 4., 2.])
        model = self.build(self.course(box_c=[centre], box_u=[u[:2]], box_h=[[2, .25, 1]]))
        hit = model.cast(centre - 5 * u, [2 * u])
        np.testing.assert_allclose(hit.t, [1.5])
        np.testing.assert_allclose(hit.normals[0], -u)

    def test_parallel_tangent_miss_and_finite_height(self):
        model = self.build()
        hit = model.cast([0., .5, 1.], [[1, 0, 0], [0, 1, 0], [-1, 0, 0], [1, 0, 1]])
        np.testing.assert_allclose(hit.t, [2, np.nan, np.nan, np.nan], equal_nan=True)
        np.testing.assert_allclose(hit.normals[0], [-1, 0, 0])
        self.assertTrue(np.isnan(hit.normals[1:]).all())
        self.assertEqual(hit.labels.tolist(), ["box:0:wall", None, None, None])
        self.assertTrue(np.isnan(model.cast([0, .6, 1], [[1, 0, 0]]).t[0]))

    def test_start_inside_returns_exit_and_flag(self):
        hit = self.build().cast([3, 0, 1], [[2, 0, 0], [0, 0, -2]])
        np.testing.assert_allclose(hit.t, [.5, .5])
        np.testing.assert_allclose(hit.normals, [[1, 0, 0], [0, 0, -1]])
        self.assertTrue(hit.origin_inside_solid)
        self.assertEqual(hit.inside_box_labels, ("box:0:wall",))

    def test_window_decomposition_preserves_opening_and_top_faces(self):
        # Four boxes surround a y/z opening, with the wall's thickness along x.
        course = self.course(box_c=[[3, -1.5, 2], [3, 1.5, 2], [3, 0, .5], [3, 0, 3.5]],
                             box_u=[[1, 0]] * 4,
                             box_h=[[.1, .5, 2], [.1, .5, 2], [.1, 1, .5], [.1, 1, .5]],
                             box_names=["window"] * 4)
        model = self.build(course)
        self.assertTrue(np.isnan(model.cast([0, 0, 2], [[1, 0, 0]]).t[0]))
        hit = model.cast([0, 0, .5], [[1, 0, 0]])
        np.testing.assert_allclose(hit.t, [2.9])
        self.assertEqual(hit.labels[0], "box:2:window")
        hit = model.cast([3, 0, 5], [[0, 0, -2]])
        np.testing.assert_allclose(hit.t, [.5])
        np.testing.assert_allclose(hit.normals, [[0, 0, 1]])

    def test_zero_parameter_excluded_at_surface(self):
        hit = self.build().cast([2, 0, 1], [[-1, 0, 0], [1, 0, 0]])
        np.testing.assert_allclose(hit.t, [np.nan, 2], equal_nan=True)
        self.assertTrue(hit.origin_inside_solid)

    def test_explicit_net_four_finite_faces_without_roof(self):
        model = self.empty(net_bounds=(-2, 3, -4, 5, 0, 2))
        hit = model.cast([0, 0, 1], [[1, 0, 0], [-1, 0, 0], [0, 1, 0],
                                    [0, -1, 0], [0, 0, 1], [1, 0, 1]])
        np.testing.assert_allclose(hit.t, [3, 2, 5, 4, np.nan, np.nan], equal_nan=True)
        np.testing.assert_allclose(hit.normals[:4], [[1, 0, 0], [-1, 0, 0],
                                                   [0, 1, 0], [0, -1, 0]])
        self.assertEqual(hit.labels[:4].tolist(), ["explicit_net:xmax", "explicit_net:xmin",
                                                  "explicit_net:ymax", "explicit_net:ymin"])
        self.assertFalse(hit.origin_inside_solid)
        self.assertTrue(np.isnan(model.cast([0, 0, 3], [[1, 0, 0]]).t[0]))

    def test_explicit_floor_infinite_plane_and_nearest_occluder(self):
        hit = self.empty(floor_z=-1).cast([0, 0, 1], [[2, 0, -2], [0, 0, 1]])
        np.testing.assert_allclose(hit.t, [1, np.nan], equal_nan=True)
        np.testing.assert_allclose(hit.normals[0], [0, 0, 1])
        self.assertEqual(hit.labels[0], "explicit_floor")
        self.assertNotIn("floor", hit.omitted_geometry)
        hit = self.build(net_bounds=(-10, 10, -10, 10, 0, 3)).cast([0, 0, 1], [[1, 0, 0]])
        np.testing.assert_allclose(hit.t, [2])
        self.assertEqual(hit.labels[0], "box:0:wall")

    def test_no_implicit_bounds_and_omissions_are_explicit(self):
        model = self.empty()
        hit = model.cast([0, 0, 1], [[1, 0, 0]])
        self.assertTrue(np.isnan(hit.t[0]))
        self.assertIn("boulder cylinders", hit.omitted_geometry)
        self.assertIn("tube curved shell", hit.omitted_geometry)
        self.assertIn("net boundary", hit.omitted_geometry)
        self.assertIn("floor", hit.omitted_geometry)
        self.assertEqual(model.published_provenance, "analytic published fixture")
        self.assertEqual(hit.source_scope, "published_course_boxes")

    def test_copied_geometry_and_empty_ray_batch(self):
        course = self.course(box_c=np.array([[3., 0., 1.]]))
        model = self.build(course)
        course.box_c[:] = 100
        np.testing.assert_allclose(model.cast([0, 0, 1], [[1, 0, 0]]).t, [2])
        hit = model.cast([0, 0, 1], np.empty((0, 3)))
        self.assertEqual(hit.t.shape, (0,))
        self.assertEqual(hit.normals.shape, (0, 3))
        self.assertFalse(model.centres.flags.writeable)

    def test_invalid_geometry_and_provenance(self):
        for values in (dict(box_c=[[0, 0]]), dict(box_h=[[0, 1, 1]]),
                       dict(box_h=[[-1, 1, 1]]), dict(box_u=[[2, 0]]),
                       dict(box_c=[[np.inf, 0, 0]]), dict(box_names=[]),
                       dict(box_c=[[1e300, 0, 0]], box_h=[[1, 1, 1]])):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.build(self.course(**values))
        for bounds in ((0, 1, 0, 1), (1, 0, 0, 1, 0, 1), (0, 1, 0, 1, 2, 2),
                       (0, 1, 0, 1, 0, np.inf)):
            with self.subTest(bounds=bounds), self.assertRaises(ValueError):
                self.empty(net_bounds=bounds)
        for floor in (np.nan, np.inf, [0]):
            with self.subTest(floor=floor), self.assertRaises(ValueError):
                self.empty(floor_z=floor)
        for provenance in (None, "", "  "):
            with self.subTest(provenance=provenance), self.assertRaises(ValueError):
                build_published_raycast(self.course(), published_provenance=provenance)

    def test_invalid_rays_and_wrapper(self):
        model = self.build()
        for origin, rays in (([0, 0], [[1, 0, 0]]), ([0, 0, np.nan], [[1, 0, 0]]),
                             ([0, 0, 0], [1, 0, 0]), ([0, 0, 0], [[0, 0, 0]]),
                             ([0, 0, 0], [[1, np.inf, 0]])):
            with self.subTest(origin=origin, rays=rays), self.assertRaises(ValueError):
                model.cast(origin, rays)
        hit = raycast_published_course(self.course(), [0, 0, 1], [[1, 0, 0]],
                                       published_provenance="analytic fixture")
        np.testing.assert_allclose(hit.t, [2])
