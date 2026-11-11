from types import SimpleNamespace
import unittest

try:
    import numpy as np
    from sim.snapshots import decode_image
except ImportError:
    np = None


def test_uint16_depth_preserves_units_and_endian():
    from types import SimpleNamespace
    import numpy as np
    message = SimpleNamespace(encoding='16UC1', width=2, height=1,
                              step=6, is_bigendian=True,
                              data=bytes([0, 0, 3, 232, 99, 99]))
    result = decode_image(message)
    np.testing.assert_array_equal(result, [[0, 1000]])
    assert result.dtype.kind == 'u'


@unittest.skipUnless(np is not None, "optional simulator NumPy not installed")
class ImageBufferTests(unittest.TestCase):
    def test_padding_is_not_mistaken_for_pixels(self):
        image = SimpleNamespace(encoding="mono8", width=2, height=2, step=4,
                                is_bigendian=False, data=bytes([1, 2, 99, 99, 3, 4, 99, 99]))
        self.assertEqual(decode_image(image).tolist(), [[1, 2], [3, 4]])

    def test_depth_endianness_and_invalid_values_are_preserved(self):
        payload = np.array([1.25, float("nan")], dtype=">f4").tobytes()
        image = SimpleNamespace(encoding="32FC1", width=2, height=1, step=8,
                                is_bigendian=True, data=payload)
        decoded = decode_image(image)
        self.assertEqual(decoded[0, 0], 1.25)
        self.assertTrue(np.isnan(decoded[0, 1]))

    def test_truncated_image_is_rejected(self):
        image = SimpleNamespace(encoding="rgb8", width=2, height=2, step=6,
                                is_bigendian=False, data=bytes(5))
        with self.assertRaises(ValueError):
            decode_image(image)
