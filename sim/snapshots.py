"""Decode ROS image buffers for simulator inspection; never infer drone state."""

import numpy as np


def decode_image(message):
    encoding = message.encoding.lower()
    formats = {"mono8": (np.dtype("u1"), 1), "rgb8": (np.dtype("u1"), 3),
               "bgr8": (np.dtype("u1"), 3),
               "16uc1": (np.dtype(">u2" if message.is_bigendian else "<u2"), 1),
               "32fc1": (np.dtype(">f4" if message.is_bigendian else "<f4"), 1)}
    if encoding not in formats:
        raise ValueError(f"unsupported inspection encoding: {encoding}")
    dtype, channels = formats[encoding]
    row_bytes = message.width * channels * dtype.itemsize
    if message.step < row_bytes or len(message.data) != message.height * message.step:
        raise ValueError("image dimensions/stride do not match its buffer")
    data = np.frombuffer(message.data, dtype="u1").reshape(message.height, message.step)
    image = np.ascontiguousarray(data[:, :row_bytes]).view(dtype)
    image = image.reshape(message.height, message.width, channels)
    if channels == 1:
        return image[:, :, 0]
    return image[:, :, ::-1].copy() if encoding == "bgr8" else image
