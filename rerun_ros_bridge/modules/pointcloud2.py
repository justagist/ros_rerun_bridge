from __future__ import annotations

from typing import Any

from numpy.lib.recfunctions import structured_to_unstructured
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY


@REGISTRY.register("pointcloud2")
class PointCloud2Module(TopicToComponentModule):
    @classmethod
    def ros_msg_type(cls):
        return PointCloud2

    def _extract_time(self, msg) -> Time | None:  # type: ignore[override]
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Any) -> None:
        # Configure color extraction (module extra supports multiple patterns)
        color_mode = self.extra.get("color_mode", "rgb_packed")  # rgb_packed | fields | none
        field_names = ["x", "y", "z"]
        pts_iter = point_cloud2.read_points(msg, field_names=field_names, skip_nans=True)
        pts = structured_to_unstructured(pts_iter)

        colors = None
        if color_mode == "rgb_packed":
            # Some drivers pack RGB into a float field named 'rgb'. We'll reinterpret offsets.
            msg.fields = [
                PointField(name="r", offset=16, datatype=PointField.UINT8, count=1),
                PointField(name="g", offset=17, datatype=PointField.UINT8, count=1),
                PointField(name="b", offset=18, datatype=PointField.UINT8, count=1),
            ]
            colors_iter = point_cloud2.read_points(msg, field_names=["r", "g", "b"], skip_nans=True)
            colors = structured_to_unstructured(colors_iter)
        elif color_mode == "fields":
            cfields = self.extra.get("color_fields", ["r", "g", "b"])
            colors_iter = point_cloud2.read_points(msg, field_names=cfields, skip_nans=True)
            colors = structured_to_unstructured(colors_iter)
        # else: no colors

        rr.log(self.entity_path, rr.Points3D(pts, colors=colors))

        stamp = self._extract_time(msg)
        for tfp in self.extra.get("tf_paths", []):
            self.context.tf.log_tf_path(
                path=tfp["path"],
                child_frame=tfp["child_frame"],
                parent_frame=tfp["parent_frame"],
                time=stamp,
            )
