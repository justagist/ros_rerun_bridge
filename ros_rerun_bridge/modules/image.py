from __future__ import annotations

# Use cv_bridge for conversion
import cv_bridge
from rclpy.time import Time
from sensor_msgs.msg import Image

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import StrictParamsModel
from ..utils.utils import IMAGE_PLANE_SUFFIX


class ImageParams(StrictParamsModel):
    """Parameters for the ImageModule."""

    encoding_override: str | None = None


@REGISTRY.register("image")
class ImageModule(TopicToComponentModule):
    """Module for handling Image messages and logging images to Rerun.

    Module-specific extra configuration parameters:
      - encoding_override: Optional encoding to override the image message's encoding.
      Options are those supported by cv_bridge. See sensor_msgs/image_encodings.h
    """

    PARAMS = ImageParams
    params: ImageParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):  # noqa: D102
        return Image

    def _extract_time(self, msg: Image) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Image) -> None:  # noqa: D102
        cvb = cv_bridge.CvBridge()
        img = (
            cvb.imgmsg_to_cv2(msg, desired_encoding=self.params.encoding_override)
            if self.params.encoding_override
            else cvb.imgmsg_to_cv2(msg)
        )
        # Shown in 3D through the camera's pinhole, logged by the camera_info module
        frame = msg.header.frame_id
        self.log(rr.Image(img), frame_id=f"{frame}{IMAGE_PLANE_SUFFIX}" if frame else "")
