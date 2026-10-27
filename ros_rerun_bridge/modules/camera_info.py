from __future__ import annotations

import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..utils.utils import IMAGE_PLANE_SUFFIX


@REGISTRY.register("camera_info")
class CameraInfoModule(TopicToComponentModule):
    """Module for handling CameraInfo messages and logging camera intrinsics to Rerun."""

    @classmethod
    def ros_msg_type(cls):
        return CameraInfo

    def _extract_time(self, msg: CameraInfo) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: CameraInfo) -> None:
        # The pinhole links the camera frame to its image plane frame, where the image modules place images
        frame = msg.header.frame_id
        rr.log(
            self.entity_path,
            rr.Pinhole(
                resolution=[msg.width, msg.height],
                image_from_camera=np.reshape(msg.k, (3, 3)),
                parent_frame=frame or None,
                child_frame=f"{frame}{IMAGE_PLANE_SUFFIX}" if frame else None,
            ),
        )
