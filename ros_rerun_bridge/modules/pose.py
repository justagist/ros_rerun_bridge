from __future__ import annotations

from typing import Any

import numpy as np
from geometry_msgs.msg import PoseStamped
from pydantic import Field
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation

import rerun as rr

from ..base import ModuleConfig, TopicToComponentModule
from ..context import BridgeContext
from ..registry import REGISTRY
from ..types import Rgba, StrictParamsModel


class PoseParams(StrictParamsModel):
    """Parameters for the PoseModule."""

    axis_length: float = Field(default=0.3, gt=0, allow_inf_nan=False)
    """Length of the pose's x (red), y (green) and z (blue) axes, in metres."""
    n_sigma: float | None = Field(default=2.0, gt=0, allow_inf_nan=False)
    """Draw the position covariance as an ellipsoid reaching this many standard deviations either side of the pose,
    along each axis. None to not draw it. Only for messages with a covariance."""
    covariance_colour: Rgba = (255, 0, 255, 255)
    """RGBA colour of the covariance ellipsoid, as uint8."""


def quaternion_xyzw(orientation: Any) -> list[float]:
    """A ROS quaternion as [x, y, z, w], with an all-zero one, common in hand-written publishers, as identity."""
    q = [orientation.x, orientation.y, orientation.z, orientation.w]
    return q if any(q) else [0.0, 0.0, 0.0, 1.0]


def covariance_ellipsoid(covariance: Any, n_sigma: float) -> tuple[list[float], list[float]]:
    """The half sizes and orientation (xyzw) of a pose's position covariance ellipsoid.

    Args:
        covariance: The pose's row-major 6x6 covariance, of x, y, z and the rotations about them.
        n_sigma: How many standard deviations the ellipsoid reaches either side of its centre, along each axis.
    """
    position = np.reshape(np.asarray(covariance, dtype=np.float64), (6, 6))[:3, :3]
    variances, axes = np.linalg.eigh(position)
    # A rotation has determinant 1: flip one axis if the eigenvectors are left-handed
    if np.linalg.det(axes) < 0:
        axes[:, 0] = -axes[:, 0]
    half_sizes = n_sigma * np.sqrt(np.clip(variances, 0.0, None))
    return half_sizes.tolist(), Rotation.from_matrix(axes).as_quat().tolist()


@REGISTRY.register("pose")
class PoseModule(TopicToComponentModule):
    """Module for drawing a pose as axes, and its position covariance as an ellipsoid if the message has one.

    Handles geometry_msgs/PoseStamped by default. For PoseWithCovarianceStamped, or nav_msgs/Odometry's pose, set
    `msg_type`. The covariance is drawn on `<entity_path>/covariance`.
    """

    PARAMS = PoseParams
    params: PoseParams  # type hint for self.params

    def __init__(self, node: Node, config: ModuleConfig, context: BridgeContext) -> None:
        """Subscribe, and check the message type is stamped.

        Args:
            node: The ROS node to use for subscriptions & logging.
            config: The module configuration.
            context: Shared context for all modules.
        """
        super().__init__(node, config, context)
        msg_type = self._resolve_msg_type()
        # The pose is drawn in its header's frame, at its stamp
        if not hasattr(msg_type(), "header"):
            raise ValueError(f"{msg_type.__name__} has no header: use a stamped type, e.g. PoseStamped")

    @classmethod
    def ros_msg_type(cls):
        return PoseStamped

    def _extract_time(self, msg: Any) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Any) -> None:
        # PoseWithCovarianceStamped and Odometry hold the pose and its covariance one level down
        with_covariance = hasattr(msg.pose, "covariance")
        pose = msg.pose.pose if with_covariance else msg.pose
        p = pose.position
        frame = msg.header.frame_id
        # The pose places its own frame in the header frame, where its axes are drawn
        pose_frame = f"pose:{self.entity_path}"
        self.log(
            rr.Transform3D(
                translation=[p.x, p.y, p.z],
                quaternion=quaternion_xyzw(pose.orientation),
                parent_frame=frame or None,
                child_frame=pose_frame,
            ),
            rr.TransformAxes3D(axis_length=self.params.axis_length),
            frame_id=pose_frame,
        )
        if with_covariance and self.params.n_sigma is not None:
            half_sizes, quaternion = covariance_ellipsoid(msg.pose.covariance, self.params.n_sigma)
            ellipsoid = rr.Ellipsoids3D(
                centers=[[p.x, p.y, p.z]],
                half_sizes=[half_sizes],
                quaternions=[quaternion],
                colors=[self.params.covariance_colour],
            )
            # In the header frame, like the pose's position
            archetypes = [ellipsoid, rr.CoordinateFrame(frame=frame)] if frame else [ellipsoid]
            rr.log(f"{self.entity_path}/covariance", *archetypes)
