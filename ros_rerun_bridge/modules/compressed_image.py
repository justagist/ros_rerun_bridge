from __future__ import annotations

from rclpy.time import Time
from sensor_msgs.msg import CompressedImage

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..utils.utils import IMAGE_PLANE_SUFFIX


def media_type_from_format(image_format: str) -> str:
    """Return the media type of a sensor_msgs/CompressedImage, from its `format` field.

    Args:
        image_format: e.g. "jpeg", "png", or image_transport's "bgr8; jpeg compressed bgr8".

    Returns:
        The media type, for Rerun to decode the image with.

    Raises:
        ValueError: If the format is not plain JPEG or PNG, e.g. compressedDepth, which adds its own header.
    """
    lowered = image_format.lower()
    if "compresseddepth" not in lowered:
        if "jpeg" in lowered or "jpg" in lowered:
            return "image/jpeg"
        if "png" in lowered:
            return "image/png"
    raise ValueError(f"unsupported compressed image format '{image_format}'")


@REGISTRY.register("compressed_image")
class CompressedImageModule(TopicToComponentModule):
    """Module for logging JPEG and PNG CompressedImage messages to Rerun as is, for the viewer to decode."""

    @classmethod
    def ros_msg_type(cls):
        return CompressedImage

    def _extract_time(self, msg: CompressedImage) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: CompressedImage) -> None:
        image = rr.EncodedImage(contents=bytes(msg.data), media_type=media_type_from_format(msg.format))
        # Shown in 3D through the camera's pinhole, logged by the camera_info module
        frame = msg.header.frame_id
        self.log(image, frame_id=f"{frame}{IMAGE_PLANE_SUFFIX}" if frame else "")
