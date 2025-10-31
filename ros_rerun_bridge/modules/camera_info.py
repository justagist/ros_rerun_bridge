from __future__ import annotations

from image_geometry import PinholeCameraModel
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY


@REGISTRY.register("camera_info")
class CameraInfoModule(TopicToComponentModule):
    """Module for handling CameraInfo messages and logging camera intrinsics to Rerun."""

    def __init__(self, *a, **kw) -> None:
        """Initialize the CameraInfoModule with a PinholeCameraModel."""
        super().__init__(*a, **kw)
        self.cam_model = PinholeCameraModel()

    @classmethod
    def ros_msg_type(cls):  # noqa: D102
        return CameraInfo

    def _extract_time(self, msg: CameraInfo) -> Time | None:  # type: ignore[override]
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: CameraInfo) -> None:  # noqa: D102
        self.cam_model.from_camera_info(msg)
        rr.log(
            self.entity_path,
            rr.Pinhole(
                resolution=[self.cam_model.width, self.cam_model.height],
                image_from_camera=self.cam_model.intrinsicMatrix(),
            ),
        )
