from __future__ import annotations

from typing import Any, Literal

import laser_geometry
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from sensor_msgs_py import point_cloud2

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import BaseModelWithTFPaths


class LaserScanParams(BaseModelWithTFPaths):
    mode: Literal["lines", "points"] = "lines"
    origin_scale: float = 0.3
    radius: float = 0.0025


@REGISTRY.register("laserscan")
class LaserScanModule(TopicToComponentModule):
    PARAMS = LaserScanParams
    params: LaserScanParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):
        return LaserScan

    def _extract_time(self, msg) -> Time | None:  # type: ignore[override]
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Any) -> None:
        proj = laser_geometry.laser_geometry.LaserProjection()
        cloud = proj.projectLaser(msg)
        pts_iter = point_cloud2.read_points(cloud, field_names=["x", "y", "z"], skip_nans=True)
        pts = structured_to_unstructured(pts_iter)

        # Optional rendering mode via extra: lines | points
        if self.params.mode == "lines":
            origin = (pts / np.linalg.norm(pts, axis=1).reshape(-1, 1)) * self.params.origin_scale
            segs = np.hstack([origin, pts]).reshape(pts.shape[0] * 2, 3)
            rr.log(self.entity_path, rr.LineStrips3D(segs, radii=self.params.radius))
        else:
            rr.log(self.entity_path, rr.Points3D(pts))

        stamp = self._extract_time(msg)
        for tfp in self.params.tf_paths:
            self.context.tf.log_tf_path(
                tf_path=tfp,
                time=stamp,
            )
