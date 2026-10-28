from __future__ import annotations

import numpy as np
from geometry_msgs.msg import PoseArray
from pydantic import Field
from rclpy.time import Time
from scipy.spatial.transform import Rotation

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import Rgba, StrictParamsModel
from .pose import quaternion_xyzw


class PoseArrayParams(StrictParamsModel):
    """Parameters for the PoseArrayModule."""

    arrow_length: float = Field(default=0.3, gt=0, allow_inf_nan=False)
    """Length of each pose's arrow, in metres."""
    radius: float = Field(default=0.01, gt=0, allow_inf_nan=False)
    """Arrow shaft radius, in metres."""
    colour: Rgba = (255, 25, 25, 255)
    """RGBA colour of the arrows, as uint8."""


@REGISTRY.register("pose_array")
class PoseArrayModule(TopicToComponentModule):
    """Module for drawing a geometry_msgs/PoseArray as an arrow per pose, along its x axis, as rviz does."""

    PARAMS = PoseArrayParams
    params: PoseArrayParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):
        return PoseArray

    def _extract_time(self, msg: PoseArray) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: PoseArray) -> None:
        if not msg.poses:
            self.log(rr.Arrows3D.cleared(), frame_id=msg.header.frame_id)
            return
        origins = np.array([[p.position.x, p.position.y, p.position.z] for p in msg.poses])
        rotations = Rotation.from_quat([quaternion_xyzw(p.orientation) for p in msg.poses])
        vectors = rotations.apply([self.params.arrow_length, 0.0, 0.0])
        arrows = rr.Arrows3D(origins=origins, vectors=vectors, radii=self.params.radius, colors=self.params.colour)
        self.log(arrows, frame_id=msg.header.frame_id)
