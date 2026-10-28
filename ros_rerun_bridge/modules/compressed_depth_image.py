from __future__ import annotations

from rclpy.time import Time
from sensor_msgs.msg import CompressedImage

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..utils.utils import IMAGE_PLANE_SUFFIX
from .depth_image import DepthParams

# compressed_depth_image_transport puts a config header (an int32 and two float32) before the compressed image
_CONFIG_HEADER_BYTES = 12
_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def encoded_depth_from_msg(msg: CompressedImage) -> tuple[bytes, str]:
    """Return a compressedDepth image's data, as Rerun decodes it, and its media type.

    Rerun reads ROS's RVL data as it is, header included, for 16UC1 and 32FC1. A PNG has its header removed. 32FC1
    PNGs are not supported: they hold quantised inverse depth, which Rerun would show as wrong distances.

    Args:
        msg: The image, from compressed_depth_image_transport, e.g. with format "16UC1; compressedDepth png".

    Raises:
        ValueError: If the format is not a supported compressedDepth one.
    """
    encoding, _, compression = msg.format.partition(";")
    codec = compression.split()[-1].lower() if compression.split() else ""
    if "compresseddepth" not in compression.lower() or codec not in ("png", "rvl"):
        raise ValueError(f"unsupported compressed depth format '{msg.format}'")
    data = bytes(msg.data)
    encoding = encoding.strip()
    if codec == "rvl":
        if encoding not in ("16UC1", "32FC1"):
            raise ValueError(f"'{msg.format}' is not supported: RVL depth must be 16UC1 or 32FC1")
        return data, "application/rvl"
    if encoding != "16UC1":
        raise ValueError(f"'{msg.format}' is not supported: only 16UC1 depth can be sent as PNG")
    png = data[_CONFIG_HEADER_BYTES:]
    if not png.startswith(_PNG_SIGNATURE):
        raise ValueError("compressed depth data is not a PNG after its header")
    return png, "image/png"


@REGISTRY.register("compressed_depth_image")
class CompressedDepthImageModule(TopicToComponentModule):
    """Module for logging compressedDepth images, as compressed_depth_image_transport publishes them, to Rerun.

    Supports PNG (16UC1) and RVL (16UC1 and 32FC1). Logged on the camera's image plane, as the `depth_image` module
    does. Uses Rerun's EncodedDepthImage, which Rerun marks as unstable.
    """

    PARAMS = DepthParams
    params: DepthParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):
        return CompressedImage

    def _extract_time(self, msg: CompressedImage) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: CompressedImage) -> None:
        data, media_type = encoded_depth_from_msg(msg)
        # Rerun's defaults match ROS's depth units: millimetres for 16-bit depth, metres for float
        image = rr.EncodedDepthImage(blob=data, media_type=media_type, **self.params.style())
        frame = msg.header.frame_id
        self.log(image, frame_id=f"{frame}{IMAGE_PLANE_SUFFIX}" if frame else "")
