import importlib.util
import unittest
from dataclasses import replace

import numpy as np

from dronebench.stereo_depth import (
    StereoDepthConfig, _independent_ambiguity_check, rectified_stereo_depth,
)


class StereoDepthValidationTests(unittest.TestCase):
    def test_bad_calibration_and_configuration_rejected(self):
        for kwargs in (dict(fx=0), dict(fx=np.nan), dict(baseline_m=-.1),
                       dict(max_depth_m=np.inf), dict(min_depth_m=4, max_depth_m=2),
                       dict(num_disparities=17), dict(num_disparities=0),
                       dict(min_disparity=-1), dict(min_disparity=2.5),
                       dict(block_size=4), dict(uniqueness_ratio=0),
                       dict(texture_window=2), dict(min_texture_std=0),
                       dict(lr_max_error_px=0), dict(min_depth_m=.001),
                       dict(speckle_window_size=-1), dict(ambiguity_exclusion_px=.5),
                       dict(ambiguity_exclusion_px=100), dict(ambiguity_relative_margin=0),
                       dict(ambiguity_absolute_margin=np.nan)):
            values = dict(fx=200., baseline_m=.1)
            values.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                StereoDepthConfig(**values)

    def test_bad_images_rejected_before_optional_dependency(self):
        config = StereoDepthConfig(fx=200., baseline_m=.1)
        image = np.zeros((100, 320), dtype=np.uint8)
        for left, right in ((image, image[:, :-1]), (image.astype(float), image),
                            (image, image.astype(np.uint16)), (image[..., None], image),
                            (np.zeros((2, 320), np.uint8), np.zeros((2, 320), np.uint8)),
                            (np.zeros((100, 64), np.uint8), np.zeros((100, 64), np.uint8))):
            with self.assertRaises(ValueError):
                rectified_stereo_depth(left, right, config)


@unittest.skipUnless(importlib.util.find_spec("cv2"), "optional OpenCV cv2 not installed")
class StereoDepthMatchingTests(unittest.TestCase):
    config = StereoDepthConfig(fx=200., baseline_m=.1, min_depth_m=.5,
                               max_depth_m=5., num_disparities=64,
                               speckle_window_size=0)

    @staticmethod
    def plane(disparity=12):
        rng = np.random.default_rng(2026)
        left = rng.integers(0, 256, (96, 320), dtype=np.uint8)
        right = rng.integers(0, 256, left.shape, dtype=np.uint8)
        right[:, :-disparity] = left[:, disparity:]
        return left, right

    def test_random_texture_plane_has_known_disparity_and_metric_z(self):
        left, right = self.plane()
        result = rectified_stereo_depth(left, right, self.config)
        region = np.s_[12:-12, 85:240]
        self.assertGreater(result.valid[region].mean(), .9)
        self.assertAlmostEqual(float(np.nanmedian(result.disparity_px[region])), 12., delta=.1)
        self.assertAlmostEqual(float(np.nanmedian(result.depth_m[region])), 20. / 12., delta=.02)
        self.assertTrue(np.isnan(result.depth_m[~result.valid]).all())
        self.assertTrue(np.isnan(result.disparity_px[~result.valid]).all())
        self.assertEqual(result.diagnostics["right_min_disparity"], -63)
        self.assertEqual(result.diagnostics["valid_pixels"], int(result.valid.sum()))
        self.assertEqual(result.diagnostics["configuration"]["baseline_m"], .1)

    def test_flat_untextured_pair_remains_invalid(self):
        image = np.full((96, 320), 127, dtype=np.uint8)
        result = rectified_stereo_depth(image, image, self.config)
        self.assertFalse(result.valid.any())
        self.assertTrue(np.isnan(result.depth_m).all())

    def test_repeated_vertical_stripes_rejected_by_uniqueness(self):
        image = np.tile(np.array([0, 0, 255, 255], dtype=np.uint8), (96, 80))
        result = rectified_stereo_depth(image, image, self.config)
        self.assertLess(result.valid.mean(), .01)

    def test_independent_cost_rejects_both_identical_periodic_grid_phases(self):
        import cv2
        yy, xx = np.indices((96, 320))
        left = np.where((xx % 12 < 2) | (yy % 10 < 2), 230, 30).astype(np.uint8)
        right = np.roll(left, -4, axis=1)
        # Both 4 and 16 px match exactly: a global smoothness/reciprocity choice
        # of either phase cannot establish metric distance. Exercise independent
        # costs directly so earlier SGBM rejection cannot make this vacuous.
        np.testing.assert_array_equal(left[:, 85:240], right[:, 81:236])
        np.testing.assert_array_equal(left[:, 85:240], right[:, 69:224])
        for chosen in (4., 16.):
            selected = np.full(left.shape, chosen, np.float32)
            unique, gap = _independent_ambiguity_check(left, right, selected, self.config, cv2)
            self.assertFalse(unique[12:-12, 85:240].any())
            np.testing.assert_allclose(gap[12:-12, 85:240], 0., atol=1e-6)
        result = rectified_stereo_depth(left, right, self.config)
        self.assertFalse(result.valid[12:-12, 85:240].any())
        self.assertTrue(np.isnan(result.depth_m[12:-12, 85:240]).all())

    def test_independent_cost_rejects_wrong_phase_with_better_alternative(self):
        import cv2
        left, right = self.plane(disparity=12)
        selected = np.full(left.shape, 24., np.float32)
        unique, gap = _independent_ambiguity_check(left, right, selected, self.config, cv2)
        self.assertFalse(unique[12:-12, 85:240].any())
        self.assertTrue(np.all(gap[12:-12, 85:240] < -20.))

    def test_unmatched_patch_is_not_filled(self):
        left, right = self.plane()
        right[:, 100:180] = 127
        result = rectified_stereo_depth(left, right, self.config)
        self.assertFalse(result.valid[12:-12, 125:175].any())
        self.assertTrue(np.isnan(result.depth_m[12:-12, 125:175]).all())

    def test_depth_outside_requested_interval_is_not_relabelled(self):
        left, right = self.plane(disparity=12)
        config = replace(self.config, min_depth_m=2., max_depth_m=5.)
        result = rectified_stereo_depth(left, right, config)
        self.assertFalse(result.valid[12:-12, 85:240].any())

    def test_nonzero_min_disparity_signed_right_range(self):
        left, right = self.plane(disparity=12)
        config = replace(self.config, min_disparity=4, num_disparities=48)
        result = rectified_stereo_depth(left, right, config)
        self.assertEqual(result.diagnostics["right_min_disparity"], -51)
        self.assertGreater(result.valid[12:-12, 85:240].mean(), .9)
        self.assertAlmostEqual(float(np.nanmedian(result.depth_m)), 20. / 12., delta=.02)

    def test_swapped_pair_does_not_create_positive_metric_depth(self):
        left, right = self.plane()
        result = rectified_stereo_depth(right, left, self.config)
        self.assertLess(result.valid[12:-12, 85:240].mean(), .01)


if __name__ == "__main__":
    unittest.main()
