from types import SimpleNamespace as NS
import unittest

import numpy as np

from tools.ros_depth_mapper import checked_camera, decode_depth, require_fresh, transform_matrix


def header(seconds=1, frame="depth_optical"):
    return NS(stamp=NS(sec=seconds, nanosec=0), frame_id=frame)


def image():
    return NS(header=header(), width=2, height=1, encoding="16UC1", is_bigendian=True,
              step=6, data=b"\x03\xe8\x00\x00\xab\xcd")


def calibration():
    return NS(header=header(), width=2, height=1, k=[2., 0., 1., 0., 2., .5, 0., 0., 1.],
              p=[2., 0., 1., 0., 0., 2., .5, 0., 0., 0., 1., 0.],
              r=list(np.eye(3).flat), d=[0.] * 5, distortion_model="plumb_bob", binning_x=0, binning_y=0,
              roi=NS(x_offset=0, y_offset=0, width=0, height=0))


class RosDepthMapperHelpersTests(unittest.TestCase):
    def test_explicit_mm_units_big_endian_and_padding(self):
        decoded = decode_depth(image(), "millimetres")
        np.testing.assert_array_equal(decoded, [[1., 0.]])
        with self.assertRaises(ValueError):
            decode_depth(image(), "metres")

    def test_float_invalid_depth_is_preserved(self):
        msg = image()
        msg.encoding, msg.is_bigendian, msg.step = "32FC1", False, 8
        msg.data = np.array([[np.nan, np.inf]], dtype="<f4").tobytes()
        decoded = decode_depth(msg, "metres")
        self.assertTrue(np.isnan(decoded[0, 0]))
        self.assertTrue(np.isinf(decoded[0, 1]))

    def test_calibration_frame_size_and_source_stamp_must_match(self):
        self.assertEqual(checked_camera(calibration(), image(), "depth_optical", .1).fx, 2.)
        info = calibration()
        info.header.frame_id = "rgb_optical"
        with self.assertRaises(ValueError):
            checked_camera(info, image(), "depth_optical", .1)
        info = calibration()
        info.header = header(2)
        with self.assertRaises(ValueError):
            checked_camera(info, image(), "depth_optical", .1)
        info = calibration()
        info.width = 3
        with self.assertRaises(ValueError):
            checked_camera(info, image(), "depth_optical", .1)

    def test_unrectified_or_shifted_calibration_rejected(self):
        for field, index, value in (("d", 0, .01), ("p", 3, -.1), ("r", 0, .9), ("k", 1, .1)):
            info = calibration()
            getattr(info, field)[index] = value
            with self.assertRaises(ValueError):
                checked_camera(info, image(), "depth_optical", .1)
        info = calibration()
        info.distortion_model = "equidistant"
        with self.assertRaises(ValueError):
            checked_camera(info, image(), "depth_optical", .1)

    def test_reject_missing_future_and_stale_source(self):
        require_fresh(1_000_000_000, 1_100_000_000, .2, "depth")
        for stamp, now in ((0, 1), (2_000_000_000, 1_000_000_000), (1, 2_000_000_000)):
            with self.assertRaises(ValueError):
                require_fresh(stamp, now, .2, "depth")

    def test_tf_rotation_translation_and_nonunit_quaternion(self):
        tf = NS(rotation=NS(x=0., y=0., z=np.sqrt(.5), w=np.sqrt(.5)),
                translation=NS(x=2., y=3., z=4.))
        result = transform_matrix(tf) @ np.array([1., 0., 0., 1.])
        np.testing.assert_allclose(result, [2., 4., 4., 1.], atol=1e-10)
        tf.rotation.w = 2.
        with self.assertRaises(ValueError):
            transform_matrix(tf)
