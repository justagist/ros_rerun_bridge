from __future__ import annotations

from typing import Literal

import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import StrictParamsModel


class PointCloud2Params(StrictParamsModel):
    """Parameters for the PointCloud2Module."""

    colour_mode: Literal["auto", "rgb_packed", "fields", "none"] = "auto"
    """Where point colours come from:
    - "auto": the packed `rgb` field if the cloud has one, otherwise no colour.
    - "rgb_packed": colour packed in a field named `rgb`, as PCL does.
    - "fields": separate channels, named in `colour_fields`.
    - "none": no colour.
    """
    colour_fields: list[str] = ["r", "g", "b"]
    """List of field names to extract colors from when `colour_mode` is "fields"."""


@REGISTRY.register("pointcloud2")
class PointCloud2Module(TopicToComponentModule):
    """Module for logging PointCloud2 messages to Rerun as points, with their colours."""

    PARAMS = PointCloud2Params
    params: PointCloud2Params  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):
        return PointCloud2

    def _extract_time(self, msg: PointCloud2) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: PointCloud2) -> None:
        mode = self.params.colour_mode
        if mode == "auto":
            mode = "rgb_packed" if any(field.name == "rgb" for field in msg.fields) else "none"
        colour_fields = {"rgb_packed": ["rgb"], "fields": self.params.colour_fields, "none": []}[mode]
        # Read in one pass so colours stay aligned with points. NaNs are filtered on xyz only: a packed RGBA float
        # with full alpha is itself a NaN bit pattern.
        cloud = point_cloud2.read_points(msg, field_names=["x", "y", "z", *colour_fields])
        pts = structured_to_unstructured(cloud[["x", "y", "z"]])
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]

        colors = None
        if mode == "rgb_packed":
            # PCL packs colour as 0x00RRGGBB in the 4 bytes of the 'rgb' field
            rgb = cloud["rgb"][valid]
            packed = rgb.astype(rgb.dtype.newbyteorder("=")).view(np.uint32)
            colors = np.stack([(packed >> 16) & 0xFF, (packed >> 8) & 0xFF, packed & 0xFF], axis=1).astype(np.uint8)
        elif mode == "fields":
            colors = structured_to_unstructured(cloud[self.params.colour_fields])[valid]

        self.log(rr.Points3D(pts, colors=colors), frame_id=msg.header.frame_id)
