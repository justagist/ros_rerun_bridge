from __future__ import annotations

from typing import Literal

from numpy.lib.recfunctions import structured_to_unstructured
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import BaseModelWithTFPaths


class PointCloud2Params(BaseModelWithTFPaths):
    """Parameters for the PointCloud2Module."""

    colour_mode: Literal["rgb_packed", "fields", "none"] = "rgb_packed"
    """Color extraction mode. Options:
    - "rgb_packed": Extract RGB from packed float field named 'rgb'.
    - "fields": Extract RGB from separate fields specified in `colour_fields`.
    - "none": No color extraction; points will be logged without color.
    """
    colour_fields: list[str] = ["r", "g", "b"]
    """List of field names to extract colors from when `colour_mode` is "fields"."""


@REGISTRY.register("pointcloud2")
class PointCloud2Module(TopicToComponentModule):
    """Module for handling PointCloud2 messages and logging to Rerun.

    Module-specific extra configuration parameters:
      - colour_mode: "rgb_packed" | "fields" | "none"  (default: "rgb_packed")
            - rgb_packed: extract RGB from packed float field named 'rgb'
            - fields: extract RGB from separate fields specified in `colour_fields`
            - none: no color extraction
      - colour_fields: list of field names for colors when colour_mode is "fields". (default: ["r", "g", "b"])
    """

    PARAMS = PointCloud2Params
    params: PointCloud2Params  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):  # noqa: D102
        return PointCloud2

    def _extract_time(self, msg: PointCloud2) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: PointCloud2) -> None:  # noqa: D102
        # Configure color extraction (module extra supports multiple patterns)
        field_names = ["x", "y", "z"]
        pts_iter = point_cloud2.read_points(msg, field_names=field_names, skip_nans=True)
        pts = structured_to_unstructured(pts_iter)

        colors = None
        if self.params.colour_mode == "rgb_packed":
            # Some drivers pack RGB into a float field named 'rgb'. We'll reinterpret offsets.
            msg.fields = [
                PointField(name="r", offset=16, datatype=PointField.UINT8, count=1),
                PointField(name="g", offset=17, datatype=PointField.UINT8, count=1),
                PointField(name="b", offset=18, datatype=PointField.UINT8, count=1),
            ]
            colors_iter = point_cloud2.read_points(msg, field_names=["r", "g", "b"], skip_nans=True)
            colors = structured_to_unstructured(colors_iter)
        elif self.params.colour_mode == "fields":
            colors_iter = point_cloud2.read_points(msg, field_names=self.params.colour_fields, skip_nans=True)
            colors = structured_to_unstructured(colors_iter)
        # else: no colors

        rr.log(self.entity_path, rr.Points3D(pts, colors=colors))

        stamp = self._extract_time(msg)
        for tfp in self.params.tf_paths:
            self.context.tf.log_tf_path(
                tf_path=tfp,
                time=stamp,
            )
