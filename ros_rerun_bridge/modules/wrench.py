from __future__ import annotations

from geometry_msgs.msg import WrenchStamped
from pydantic import Field
from rclpy.time import Time

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import Rgba, StrictParamsModel


class WrenchParams(StrictParamsModel):
    """Parameters for the WrenchModule."""

    force_scale: float = Field(default=0.1, gt=0, allow_inf_nan=False)
    """Length of the force arrow per newton, in metres."""
    torque_scale: float = Field(default=0.1, gt=0, allow_inf_nan=False)
    """Length of the torque arrow per newton metre, in metres."""
    force_colour: Rgba = (255, 80, 80, 255)
    """RGBA colour of the force arrow, as uint8."""
    torque_colour: Rgba = (255, 220, 50, 255)
    """RGBA colour of the torque arrow, as uint8."""
    radius: float = Field(default=0.01, gt=0, allow_inf_nan=False)
    """Arrow shaft radius, in metres."""


@REGISTRY.register("wrench")
class WrenchModule(TopicToComponentModule):
    """Module for drawing a geometry_msgs/WrenchStamped as force and torque arrows from its frame's origin.

    The torque is drawn along its axis, by the right-hand rule, rather than as rviz's circular arrow.
    """

    PARAMS = WrenchParams
    params: WrenchParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):
        return WrenchStamped

    def _extract_time(self, msg: WrenchStamped) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: WrenchStamped) -> None:
        f, t = msg.wrench.force, msg.wrench.torque
        arrows = rr.Arrows3D(
            origins=[[0.0, 0.0, 0.0]] * 2,
            vectors=[
                [f.x * self.params.force_scale, f.y * self.params.force_scale, f.z * self.params.force_scale],
                [t.x * self.params.torque_scale, t.y * self.params.torque_scale, t.z * self.params.torque_scale],
            ],
            radii=self.params.radius,
            colors=[self.params.force_colour, self.params.torque_colour],
            labels=["force", "torque"],
        )
        self.log(arrows, frame_id=msg.header.frame_id)
