from __future__ import annotations

from typing import Literal

import numpy as np
from pydantic import Field
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import StrictParamsModel
from ..utils.utils import IMAGE_PLANE_SUFFIX


class CameraInfoParams(StrictParamsModel):
    """Parameters for the CameraInfoModule."""

    intrinsics: Literal["k", "p"] = "k"
    """Which intrinsics match the images shown: "k" for raw images, "p" (its left 3x3) for rectified ones."""
    image_plane_distance: float | None = Field(default=None, gt=0, allow_inf_nan=False)
    """How far in front of the camera the image plane is drawn in 3D, in metres. One from the scene's size if unset."""


@REGISTRY.register("camera_info")
class CameraInfoModule(TopicToComponentModule):
    """Module for handling CameraInfo messages and logging camera intrinsics to Rerun."""

    PARAMS = CameraInfoParams
    params: CameraInfoParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):
        return CameraInfo

    def _extract_time(self, msg: CameraInfo) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: CameraInfo) -> None:
        # The intrinsics are for the full sensor resolution: adjust them, and the resolution, to the published image's
        # binning and region of interest, as image_geometry does
        binning_x, binning_y = max(1, msg.binning_x), max(1, msg.binning_y)
        roi = msg.roi
        full_frame = roi.x_offset == roi.y_offset == roi.width == roi.height == 0
        width, height = (msg.width, msg.height) if full_frame else (roi.width, roi.height)
        if self.params.intrinsics == "k":
            k = np.reshape(msg.k, (3, 3)).astype(np.float64)
        else:
            k = np.reshape(msg.p, (3, 4))[:, :3].astype(np.float64)
        k[0, 0] /= binning_x
        k[1, 1] /= binning_y
        k[0, 2] = (k[0, 2] - roi.x_offset) / binning_x
        k[1, 2] = (k[1, 2] - roi.y_offset) / binning_y

        # The pinhole links the camera frame to its image plane frame, where the image modules place images
        frame = msg.header.frame_id
        rr.log(
            self.entity_path,
            rr.Pinhole(
                resolution=[width // binning_x, height // binning_y],
                image_from_camera=k,
                parent_frame=frame or None,
                child_frame=f"{frame}{IMAGE_PLANE_SUFFIX}" if frame else None,
                image_plane_distance=self.params.image_plane_distance,
            ),
        )
