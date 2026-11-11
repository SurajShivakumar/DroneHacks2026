import unittest
from dataclasses import replace

import numpy as np

from dronebench.depth_wall_constraint import WallConstraintConfig, extract_wall_constraints


class DepthWallConstraintTests(unittest.TestCase):
    walls = np.array([[[3., -3.], [3., 5.]], [[-3., 4.], [3., 4.]]])
    pose = np.array([.4, .2, .1])

    def points(self, world):
        c, s = np.cos(self.pose[2]), np.sin(self.pose[2])
        return (np.asarray(world) - self.pose[:2]) @ np.array([[c, -s], [s, c]])

    def vertical_points(self):
        return self.points(np.column_stack((np.full(60, 3.), np.linspace(-2., 3.5, 60))))

    def test_single_wall_observes_only_normal_projection(self):
        seed = self.pose + [.08, -.07, 0.]
        result = extract_wall_constraints(self.vertical_points(), self.walls[:1], seed)
        self.assertEqual(len(result.constraints), 1)
        factor = result.constraints[0]
        self.assertTrue(factor.accepted, factor.reason)
        np.testing.assert_allclose(factor.normal, [1., 0.])
        self.assertAlmostEqual(factor.projection, self.pose[0])
        self.assertAlmostEqual(factor.innovation, -.08)
        self.assertEqual(np.linalg.matrix_rank(np.outer(factor.normal, factor.normal) / factor.variance), 1)
        self.assertGreaterEqual(factor.variance, .03 ** 2)
        # Move the seed tangentially: the factor still contains only x.
        shifted = extract_wall_constraints(self.vertical_points(), self.walls[:1], seed + [0., .4, 0.])
        self.assertAlmostEqual(shifted.constraints[0].projection, factor.projection)
        self.assertFalse(hasattr(factor, "pose"))

    def test_perpendicular_walls_return_two_scalar_factors(self):
        horizontal = self.points(np.column_stack((np.linspace(-2., 2.5, 60), np.full(60, 4.))))
        result = extract_wall_constraints(np.vstack((self.vertical_points(), horizontal)), self.walls,
                                          self.pose + [.08, -.07, 0.])
        accepted = [f for f in result.constraints if f.accepted]
        self.assertEqual(len(accepted), 2)
        for factor in accepted:
            self.assertAlmostEqual(factor.projection, np.dot(factor.normal, self.pose[:2]))

    def test_duplicate_pixels_do_not_increase_support_or_confidence(self):
        points = self.vertical_points()
        a = extract_wall_constraints(points, self.walls[:1], self.pose).constraints[0]
        b = extract_wall_constraints(np.repeat(points, 50, axis=0), self.walls[:1], self.pose).constraints[0]
        self.assertEqual(a.inliers, b.inliers)
        self.assertEqual(a.variance, b.variance)

    def test_ambiguous_parallel_planes_rejected(self):
        walls = np.array([[[3., -3.], [3., 5.]], [[3.2, 5.], [3.2, -3.]]])
        result = extract_wall_constraints(self.vertical_points(), walls, self.pose + [.10, 0., 0.])
        self.assertGreater(result.ambiguous_points, 0)
        factor = result.constraints[0]
        self.assertFalse(factor.accepted)
        self.assertEqual(factor.reason, "ambiguous_parallel_association")
        self.assertIsNone(factor.projection)
        self.assertIsNone(factor.variance)

    def test_reversed_coplanar_duplicate_walls_are_not_ambiguous(self):
        walls = np.concatenate((self.walls[:1], self.walls[:1, ::-1]))
        result = extract_wall_constraints(self.vertical_points(), walls, self.pose)
        self.assertEqual(result.ambiguous_points, 0)
        self.assertEqual(len(result.constraints), 1)
        self.assertTrue(result.constraints[0].accepted)

    def test_outliers_invalid_rows_and_unmatched_returns_handled(self):
        rng = np.random.default_rng(4)
        measured = self.vertical_points() + rng.normal(0., .003, (60, 2))
        local_outliers = self.points([[3.07, -.4], [3.07, .5], [3.07, 1.6]])
        points = np.vstack((measured, local_outliers, [[20., 20.]], [[np.nan, 0], [0, np.inf]]))
        result = extract_wall_constraints(points, self.walls[:1], self.pose)
        self.assertEqual(result.invalid_points, 2)
        self.assertEqual(result.finite_points, 64)
        factor = result.constraints[0]
        self.assertTrue(factor.accepted, factor.reason)
        self.assertAlmostEqual(factor.projection, self.pose[0], delta=.005)
        self.assertLess(factor.inlier_ratio, 1.)

    def test_secondary_compact_projection_mode_rejected(self):
        wrong = self.points(np.column_stack((np.full(20, 3.1), np.linspace(-1.95, 3.45, 20))))
        result = extract_wall_constraints(np.vstack((self.vertical_points(), wrong)), self.walls[:1], self.pose)
        factor = result.constraints[0]
        self.assertFalse(factor.accepted)
        self.assertEqual(factor.reason, "multimodal_projection")

    def test_out_of_bound_innovation_rejected(self):
        result = extract_wall_constraints(self.vertical_points(), self.walls[:1], self.pose + [.14, 0, 0])
        factor = result.constraints[0]
        self.assertEqual(factor.reason, "innovation_limit")
        self.assertIsNone(factor.projection)

    def test_finite_segment_endpoints_not_infinite_wall(self):
        points = self.points(np.column_stack((np.full(30, 3.), np.linspace(6., 9., 30))))
        result = extract_wall_constraints(points, self.walls[:1], self.pose)
        self.assertEqual(result.associated_points, 0)
        self.assertEqual(result.constraints, ())

    def test_yaw_uncertainty_increases_scalar_variance(self):
        a = extract_wall_constraints(self.vertical_points(), self.walls[:1], self.pose).constraints[0]
        config = replace(WallConstraintConfig(), yaw_std_rad=.06)
        b = extract_wall_constraints(self.vertical_points(), self.walls[:1], self.pose, config=config).constraints[0]
        self.assertTrue(a.accepted and b.accepted)
        self.assertGreater(b.variance, a.variance)

    def test_insufficient_independent_support_or_spread_rejected(self):
        result = extract_wall_constraints(np.repeat(self.vertical_points()[:1], 100, axis=0), self.walls[:1], self.pose)
        self.assertEqual(result.reason, "insufficient_points")
        narrow = self.points(np.column_stack((np.full(30, 3.), np.linspace(0., .2, 30))))
        config = replace(WallConstraintConfig(), sample_spacing=.005)
        result = extract_wall_constraints(narrow, self.walls[:1], self.pose, config=config)
        self.assertEqual(result.constraints[0].reason, "insufficient_spatial_spread")

    def test_invalid_inputs_rejected(self):
        for points, walls, seed in (([1, 2], self.walls, self.pose),
                                    (self.vertical_points(), np.zeros((1, 2, 2)), self.pose),
                                    (self.vertical_points(), self.walls * np.nan, self.pose),
                                    (self.vertical_points(), self.walls, [0, np.nan, 0])):
            with self.assertRaises(ValueError):
                extract_wall_constraints(points, walls, seed)
        for kwargs in ({"sample_spacing": 0}, {"yaw_std_rad": np.nan},
                       {"min_inliers": True}, {"min_inlier_ratio": 1.2}):
            with self.assertRaises(ValueError):
                WallConstraintConfig(**kwargs)


if __name__ == "__main__":
    unittest.main()
