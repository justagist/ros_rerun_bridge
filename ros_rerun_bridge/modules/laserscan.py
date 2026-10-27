from __future__ import annotations

from typing import Literal

import laser_geometry
import numpy as np
from numpy.lib.recfunctions import structured_to_unstructured
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from sensor_msgs_py import point_cloud2

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import StrictParamsModel


class LaserScanParams(StrictParamsModel):
    """Parameters for the LaserScanModule."""

    mode: Literal["lines", "points"] = "lines"
    """Rendering mode: "lines" to render as line segments, "points" to render as points."""
    origin_scale: float = 0.3
    """Scale factor for the laser origin when rendering lines. This is how far from (0,0,0) the line segments start."""
    radius: float = 0.0025
    """Radius for line segments when rendering in 'lines' mode."""


@REGISTRY.register("laserscan")
class LaserScanModule(TopicToComponentModule):
    """Module for logging LaserScan messages to Rerun as rays from the scanner, or as points."""

    PARAMS = LaserScanParams
    params: LaserScanParams  # type hint for self.params

    def __init__(self, *a, **kw) -> None:
        """Initialise the LaserScanModule with a reusable LaserProjection."""
        super().__init__(*a, **kw)
        self.projection = laser_geometry.laser_geometry.LaserProjection()

    @classmethod
    def ros_msg_type(cls):
        return LaserScan

    def _extract_time(self, msg: LaserScan) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: LaserScan) -> None:
        cloud = self.projection.projectLaser(msg)
        pts_iter = point_cloud2.read_points(cloud, field_names=["x", "y", "z"], skip_nans=True)
        pts = structured_to_unstructured(pts_iter)

        # Optional rendering mode via extra: lines | points
        if self.params.mode == "lines":
            # Rays start origin_scale out from the scanner, or at the return itself if that is nearer
            distance = np.linalg.norm(pts, axis=1, keepdims=True)
            fraction = np.ones_like(distance)
            np.divide(self.params.origin_scale, distance, out=fraction, where=distance > 0)
            origin = pts * np.minimum(1.0, fraction)
            # (N, 2, 3): one two-point strip per ray. A flat (2N, 3) array would be drawn as a single zigzag strip.
            segs = np.hstack([origin, pts]).reshape(-1, 2, 3)
            self.log(rr.LineStrips3D(segs, radii=self.params.radius), frame_id=msg.header.frame_id)
        else:
            self.log(rr.Points3D(pts), frame_id=msg.header.frame_id)
