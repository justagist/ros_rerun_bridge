from __future__ import annotations

import math

import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import Range

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import StrictParamsModel


class RangeParams(StrictParamsModel):
    """Parameters for the RangeModule."""

    colour: tuple[int, int, int, int] = (255, 255, 255, 255)
    """RGBA colour as 4-tuple of uint8."""
    radius: float = 0.001
    """Line radius in metres."""


@REGISTRY.register("range")
class RangeModule(TopicToComponentModule):
    """Module for logging sensor_msgs/Range readings to Rerun as rviz does: a cone along the sensor frame's x axis.

    The cone's apex is the sensor, its length the measured range and its angle the field of view. Readings outside
    [min_range, max_range], such as +inf "no detection" readings, clear the cone. A fixed-distance sensor's -inf
    detection is drawn at its fixed range.
    """

    PARAMS = RangeParams
    params: RangeParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):
        return Range

    def _extract_time(self, msg: Range) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Range) -> None:
        distance = msg.range
        # A fixed-distance sensor (min_range == max_range) reports a detection as -inf, at its fixed range
        if msg.min_range == msg.max_range and distance == -math.inf:
            distance = msg.max_range
        if not (math.isfinite(distance) and msg.min_range <= distance <= msg.max_range):
            rr.log(self.entity_path, rr.Clear(recursive=False))
            return
        rim_radius = distance * math.tan(msg.field_of_view / 2)
        angles = np.linspace(0, 2 * math.pi, 33)
        rim = np.column_stack(
            [np.full_like(angles, distance), rim_radius * np.cos(angles), rim_radius * np.sin(angles)]
        )
        # The rim and four lines from the sensor to it
        edges = [[[0, 0, 0], rim[i]] for i in range(0, 32, 8)]
        self.log(
            rr.LineStrips3D([rim, *edges], radii=self.params.radius, colors=self.params.colour),
            frame_id=msg.header.frame_id,
        )
