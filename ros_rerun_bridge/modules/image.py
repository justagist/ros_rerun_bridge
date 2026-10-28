from __future__ import annotations

import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import Image

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..utils.utils import IMAGE_PLANE_SUFFIX

# ROS encoding -> (channel dtype, channels, Rerun colour model). See sensor_msgs/image_encodings.hpp.
_PIXEL_ENCODINGS = {
    "rgb8": (np.uint8, 3, "RGB"),
    "bgr8": (np.uint8, 3, "BGR"),
    "rgba8": (np.uint8, 4, "RGBA"),
    "bgra8": (np.uint8, 4, "BGRA"),
    "rgb16": (np.uint16, 3, "RGB"),
    "bgr16": (np.uint16, 3, "BGR"),
    "rgba16": (np.uint16, 4, "RGBA"),
    "bgra16": (np.uint16, 4, "BGRA"),
    "mono8": (np.uint8, 1, "L"),
    "mono16": (np.uint16, 1, "L"),
    "8UC1": (np.uint8, 1, "L"),
    "16UC1": (np.uint16, 1, "L"),
    "32FC1": (np.float32, 1, "L"),
}

# ROS encodings Rerun decodes itself from the raw bytes -> (pixel format, bytes per pixel)
_PACKED_ENCODINGS = {
    "bayer_rggb8": (rr.PixelFormat.BayerRGGB8, 1),
    "bayer_bggr8": (rr.PixelFormat.BayerBGGR8, 1),
    "bayer_gbrg8": (rr.PixelFormat.BayerGBRG8, 1),
    "bayer_grbg8": (rr.PixelFormat.BayerGRBG8, 1),
    "yuv422_yuy2": (rr.PixelFormat.YUY2, 2),
}


def image_from_msg(msg: Image) -> rr.Image:
    """Convert a sensor_msgs/Image to a Rerun image, without copying through OpenCV.

    Args:
        msg: The ROS image.

    Returns:
        The Rerun image.

    Raises:
        ValueError: If the encoding is not supported.
    """
    # Rows can be padded: step is the row length in bytes
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    if msg.encoding in _PACKED_ENCODINGS:
        pixel_format, pixel_bytes = _PACKED_ENCODINGS[msg.encoding]
        packed = rows[:, : msg.width * pixel_bytes].tobytes()
        return rr.Image(bytes=packed, width=msg.width, height=msg.height, pixel_format=pixel_format)
    return rr.Image(pixels_from_msg(msg), color_model=_PIXEL_ENCODINGS[msg.encoding][2])


def pixels_from_msg(msg: Image) -> np.ndarray:
    """Return a sensor_msgs/Image's pixels as an array, without row padding and in the machine's byte order.

    Args:
        msg: The ROS image, in one of the unpacked encodings, e.g. rgb8 or 16UC1.

    Returns:
        The pixels, shaped (height, width) for one channel or (height, width, channels).

    Raises:
        ValueError: If the encoding is not supported.
    """
    if msg.encoding not in _PIXEL_ENCODINGS:
        raise ValueError(f"unsupported image encoding '{msg.encoding}'")
    dtype, channels, _ = _PIXEL_ENCODINGS[msg.encoding]
    dtype = np.dtype(dtype).newbyteorder(">" if msg.is_bigendian else "<")
    # Rows can be padded: step is the row length in bytes
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = rows[:, : msg.width * channels * dtype.itemsize].view(dtype).astype(dtype.newbyteorder("="), copy=False)
    shape = (msg.height, msg.width) if channels == 1 else (msg.height, msg.width, channels)
    return pixels.reshape(shape)


@REGISTRY.register("image")
class ImageModule(TopicToComponentModule):
    """Module for handling Image messages and logging images to Rerun.

    Supports the RGB(A), BGR(A) and mono encodings at 8 and 16 bits, 8UC1, 16UC1 and 32FC1, Bayer 8-bit and
    yuv422_yuy2.
    """

    @classmethod
    def ros_msg_type(cls):
        return Image

    def _extract_time(self, msg: Image) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Image) -> None:
        # Shown in 3D through the camera's pinhole, logged by the camera_info module
        frame = msg.header.frame_id
        self.log(image_from_msg(msg), frame_id=f"{frame}{IMAGE_PLANE_SUFFIX}" if frame else "")
