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
from ..types import BaseModelWithTFPaths


class LaserScanParams(BaseModelWithTFPaths):
    """Parameters for the LaserScanModule."""

    mode: Literal["lines", "points"] = "lines"
    """Rendering mode: "lines" to render as line segments, "points" to render as points."""
    origin_scale: float = 0.3
    """Scale factor for the laser origin when rendering lines. This is how far from (0,0,0) the line segments start."""
    radius: float = 0.0025
    """Radius for line segments when rendering in 'lines' mode."""


@REGISTRY.register("laserscan")
class LaserScanModule(TopicToComponentModule):
    """Module for handling LaserScan messages and logging to Rerun.

    Module-specific extra configuration parameters:
      - mode: "lines" | "points"  (default: "lines")
            - lines: render as line segments from origin to points
            - points: render as points
      - origin_scale: float (default: 0.3)
            - scale factor for the laser origin when rendering lines
      - radius: float (default: 0.0025)
            - radius for line segments when rendering in 'lines' mode
    """

    PARAMS = LaserScanParams
    params: LaserScanParams  # type hint for self.params

    def __init__(self, *a, **kw) -> None:
        """Initialise the LaserScanModule with a reusable LaserProjection."""
        super().__init__(*a, **kw)
        self.projection = laser_geometry.laser_geometry.LaserProjection()

    @classmethod
    def ros_msg_type(cls):  # noqa: D102
        return LaserScan

    def _extract_time(self, msg: LaserScan) -> Time | None:  # type: ignore[override]
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: LaserScan) -> None:  # noqa: D102
        cloud = self.projection.projectLaser(msg)
        pts_iter = point_cloud2.read_points(cloud, field_names=["x", "y", "z"], skip_nans=True)
        pts = structured_to_unstructured(pts_iter)

        # Optional rendering mode via extra: lines | points
        if self.params.mode == "lines":
            origin = (pts / np.linalg.norm(pts, axis=1).reshape(-1, 1)) * self.params.origin_scale
            # (N, 2, 3): one two-point strip per ray. A flat (2N, 3) array would be drawn as a single zigzag strip.
            segs = np.hstack([origin, pts]).reshape(-1, 2, 3)
            rr.log(self.entity_path, rr.LineStrips3D(segs, radii=self.params.radius))
        else:
            rr.log(self.entity_path, rr.Points3D(pts))

        stamp = self._extract_time(msg)
        for tfp in self.params.tf_paths:
            self.context.tf.log_tf_path(
                tf_path=tfp,
                time=stamp,
            )
