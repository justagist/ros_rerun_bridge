from __future__ import annotations

from typing import Any

from image_geometry import PinholeCameraModel
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY


@REGISTRY.register("camera_info")
class CameraInfoModule(TopicToComponentModule):
    def __init__(self, *a, **kw) -> None:
        super().__init__(*a, **kw)
        self.model = PinholeCameraModel()

    @classmethod
    def ros_msg_type(cls):
        return CameraInfo

    def _extract_time(self, msg) -> Time | None:  # type: ignore[override]
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Any) -> None:
        self.model.fromCameraInfo(msg)
        rr.log(
            self.entity_path,
            rr.Pinhole(
                resolution=[self.model.width, self.model.height],
                image_from_camera=self.model.intrinsicMatrix(),
            ),
        )
