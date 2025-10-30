from __future__ import annotations

from typing import Any

# Use cv_bridge for conversion
import cv_bridge
from rclpy.time import Time
from sensor_msgs.msg import Image

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY


@REGISTRY.register("image")
class ImageModule(TopicToComponentModule):
    @classmethod
    def ros_msg_type(cls):
        return Image

    def _extract_time(self, msg) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Any) -> None:
        # extra: {encoding_override: str | None}
        encoding_override = self.extra.get("encoding_override")

        cvb = cv_bridge.CvBridge()
        img = cvb.imgmsg_to_cv2(msg, desired_encoding=encoding_override) if encoding_override else cvb.imgmsg_to_cv2(msg)
        rr.log(self.entity_path, rr.Image(img))

        stamp = self._extract_time(msg)
        for tfp in self.extra.get("tf_paths", []):
            self.context.tf.log_tf_path(
                path=tfp["path"],  # entity path to attach transform on
                child_frame=tfp["child_frame"],  # TF child
                parent_frame=tfp["parent_frame"],  # TF parent
                time=stamp,
            )
