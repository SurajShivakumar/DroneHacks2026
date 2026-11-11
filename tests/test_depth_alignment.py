import unittest
from dataclasses import replace

import numpy as np

from dronebench.depth_alignment import AlignmentConfig, align_points_to_segments


class DepthAlignmentTests(unittest.TestCase):
    walls = np.array([[[3., -2.], [3., 4.]], [[-2., 4.], [3., 4.]]])
    true_pose = np.array([.4, .2, .15])

    @classmethod
    def points(cls):
        world = np.vstack((np.column_stack((np.full(60, 3.), np.linspace(-1.5, 3.5, 60))),
                           np.column_stack((np.linspace(-1.5, 2.5, 60), np.full(60, 4.)))))
        c, s = np.cos(cls.true_pose[2]), np.sin(cls.true_pose[2])
        return (world - cls.true_pose[:2]) @ np.array([[c, -s], [s, c]])

    def test_perpendicular_walls_recover_translation_and_yaw(self):
        seed = self.true_pose + [.12, -.09, .04]
        result = align_points_to_segments(self.points(), self.walls, seed)
        self.assertTrue(result.accepted, result.reason)
        np.testing.assert_allclose(result.pose, self.true_pose, atol=1e-4)
        self.assertLessEqual(result.iterations, 40)
        self.assertEqual(result.covariance.shape, (3, 3))
        self.assertTrue(np.all(np.linalg.eigvalsh(result.covariance) > 0))

    def test_single_wall_does_not_invent_tangential_position(self):
        seed = self.true_pose + [.10, -.12, .02]
        result = align_points_to_segments(self.points()[:60], self.walls[:1], seed)
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "unobservable_geometry")
        np.testing.assert_array_equal(result.pose, seed)
        self.assertIsNone(result.covariance)

    def test_parallel_walls_also_reject(self):
        walls = np.array([[[0., -4.], [0., 4.]], [[3., -4.], [3., 4.]]])
        points = np.vstack((np.column_stack((np.zeros(40), np.linspace(-3, 3, 40))),
                            np.column_stack((np.full(40, 3.), np.linspace(-3, 3, 40)))))
        result = align_points_to_segments(points, walls, [.1, .1, .02])
        self.assertEqual(result.reason, "unobservable_geometry")

    def test_far_outliers_and_invalid_sensor_rows_are_handled(self):
        rng = np.random.default_rng(14)
        points = np.vstack((self.points() + rng.normal(0, .008, (120, 2)),
                            rng.uniform(10., 20., (20, 2)),
                            [[np.nan, 0], [0, np.inf], [-np.inf, np.nan]]))
        result = align_points_to_segments(points, self.walls, self.true_pose + [.1, -.1, .03])
        self.assertTrue(result.accepted, result.reason)
        self.assertEqual(result.invalid_points, 3)
        self.assertEqual(result.finite_points, 140)
        np.testing.assert_allclose(result.pose, self.true_pose, atol=.015)

    def test_large_correction_rejected_and_seed_preserved(self):
        seed = self.true_pose + [.14, -.12, .02]
        config = replace(AlignmentConfig(), max_translation_correction=.06)
        result = align_points_to_segments(self.points(), self.walls, seed, config=config)
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "correction_limit")
        np.testing.assert_array_equal(result.pose, seed)

    def test_correlated_repeated_pixels_do_not_shrink_covariance(self):
        points = self.points()
        result = align_points_to_segments(points, self.walls, self.true_pose)
        repeated = align_points_to_segments(np.repeat(points, 100, axis=0), self.walls, self.true_pose)
        self.assertTrue(result.accepted)
        self.assertTrue(repeated.accepted)
        self.assertEqual(result.sampled_points, repeated.sampled_points)
        np.testing.assert_allclose(result.covariance, repeated.covariance)
        self.assertTrue(np.all(np.diag(result.covariance) >= np.array([.03, .03, np.deg2rad(2)]) ** 2))

    def test_reject_too_few_points_and_low_inlier_ratio(self):
        result = align_points_to_segments(np.empty((0, 2)), self.walls, self.true_pose)
        self.assertEqual(result.reason, "insufficient_points")
        rng = np.random.default_rng(9)
        points = np.vstack((self.points(), rng.uniform(10., 30., (500, 2))))
        result = align_points_to_segments(points, self.walls, self.true_pose)
        self.assertEqual(result.reason, "insufficient_inliers")

    def test_endpoints_do_not_add_false_tangential_constraints(self):
        points = np.column_stack((np.full(30, 3.01), np.linspace(-4., -2.1, 30)))
        result = align_points_to_segments(points, self.walls[:1], [0., 0., 0.])
        self.assertEqual(result.reason, "insufficient_inliers")

    def test_nonconvergence_is_rejected(self):
        result = align_points_to_segments(self.points(), self.walls,
                                         self.true_pose + [.1, -.1, .03],
                                         config=replace(AlignmentConfig(), max_iterations=1))
        self.assertEqual(result.reason, "iteration_limit")

    def test_invalid_inputs_and_settings_rejected(self):
        for seed in ([0., 0., np.nan], [0., 0.]):
            with self.assertRaises(ValueError):
                align_points_to_segments(self.points(), self.walls, seed)
        for walls in (np.zeros((1, 2, 2)), np.full((1, 2, 2), np.nan), []):
            with self.assertRaises(ValueError):
                align_points_to_segments(self.points(), walls, self.true_pose)
        for kwargs in ({"sample_spacing": 0}, {"min_inlier_ratio": 1.1},
                       {"max_iterations": True}, {"noise_floor": np.nan}):
            with self.assertRaises(ValueError):
                AlignmentConfig(**kwargs)


if __name__ == "__main__":
    unittest.main()
