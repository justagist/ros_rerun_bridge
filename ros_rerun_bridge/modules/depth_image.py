from __future__ import annotations

from typing import Literal

from rclpy.time import Time
from sensor_msgs.msg import Image

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import StrictParamsModel
from ..utils.utils import IMAGE_PLANE_SUFFIX
from .image import pixels_from_msg

# Depth encodings -> how many of their units make a metre, as ROS's depth image conventions (REP 118) set them
_DEPTH_UNITS_PER_METRE = {"16UC1": 1000.0, "mono16": 1000.0, "32FC1": 1.0}


class DepthParams(StrictParamsModel):
    """Parameters for the depth image modules."""

    colormap: Literal["Turbo", "Viridis", "Inferno", "Magma", "Plasma", "Grayscale", "CyanToYellow"] | None = None
    """Colours of the depths in 2D and in the point cloud. Rerun's default, Turbo, if unset."""
    depth_range: tuple[float, float] | None = None
    """Depths the colormap spans, in the image's units (e.g. millimetres for 16UC1). The image's own range if unset."""
    point_fill_ratio: float | None = None
    """Size of each 3D point, relative to the gap between neighbouring points. Rerun's default if unset."""

    def style(self) -> dict:
        """The params as Rerun depth image fields, leaving out those not set."""
        style = {"depth_range": self.depth_range, "point_fill_ratio": self.point_fill_ratio}
        if self.colormap is not None:
            style["colormap"] = getattr(rr.components.Colormap, self.colormap)
        return {name: value for name, value in style.items() if value is not None}


@REGISTRY.register("depth_image")
class DepthImageModule(TopicToComponentModule):
    """Module for logging 16UC1 (millimetres) and 32FC1 (metres) depth images to Rerun.

    Images are logged on the camera's image plane, so with the `camera_info` module's pinhole, Rerun also draws them as
    a point cloud in 3D.
    """

    PARAMS = DepthParams
    params: DepthParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):
        return Image

    def _extract_time(self, msg: Image) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Image) -> None:
        if msg.encoding not in _DEPTH_UNITS_PER_METRE:
            raise ValueError(f"'{msg.encoding}' is not a depth encoding: 16UC1, mono16 or 32FC1")
        image = rr.DepthImage(pixels_from_msg(msg), meter=_DEPTH_UNITS_PER_METRE[msg.encoding], **self.params.style())
        frame = msg.header.frame_id
        self.log(image, frame_id=f"{frame}{IMAGE_PLANE_SUFFIX}" if frame else "")
