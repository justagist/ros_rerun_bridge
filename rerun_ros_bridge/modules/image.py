from __future__ import annotations

from typing import Any

# Use cv_bridge for conversion
import cv_bridge
from rclpy.time import Time
from sensor_msgs.msg import Image

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import BaseModelWithTFPaths


class ImageParams(BaseModelWithTFPaths):
    encoding_override: str | None = None


@REGISTRY.register("image")
class ImageModule(TopicToComponentModule):
    PARAMS = ImageParams
    params: ImageParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):
        return Image

    def _extract_time(self, msg) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Any) -> None:
        cvb = cv_bridge.CvBridge()
        img = (
            cvb.imgmsg_to_cv2(msg, desired_encoding=self.params.encoding_override)
            if self.params.encoding_override
            else cvb.imgmsg_to_cv2(msg)
        )
        rr.log(self.entity_path, rr.Image(img))

        stamp = self._extract_time(msg)
        for tfp in self.params.tf_paths:
            self.context.tf.log_tf_path(
                tf_path=tfp,
                time=stamp,
            )
