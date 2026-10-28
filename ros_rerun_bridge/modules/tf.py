from __future__ import annotations

from pydantic import model_validator
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage

import rerun as rr

from ..base import ModuleConfig, TopicToComponentModule
from ..context import BridgeContext
from ..registry import REGISTRY
from ..types import StrictParamsModel
from ..utils.utils import set_rr_time_from_ros, stamp_or_now


class FrameAxesParams(StrictParamsModel):
    """Parameters for drawing the frames a module logs transforms for, shared by the modules that do."""

    axis_length: float | None = None
    """Draw each frame's x (red), y (green) and z (blue) axes, this long in metres, as rviz's TF display does."""
    frame_names: bool = False
    """Label each frame's axes with its name. Needs `axis_length`."""

    @model_validator(mode="after")
    def _check_axes(self) -> FrameAxesParams:
        """Frame names are drawn on the axes, so there must be axes."""
        if self.frame_names and self.axis_length is None:
            raise ValueError("frame_names needs axis_length")
        return self


class FrameAxes:
    """Draws each frame's axes, if the params ask for them, on `frames/<frame>`.

    An entity per frame lets the viewer show or hide each frame's axes, and its name, on its own. The path is the same
    for every module, so a frame that several modules see, e.g. from /tf_static and /joint_states, is drawn once.
    """

    def __init__(self, params: FrameAxesParams) -> None:
        """Set up drawing frames.

        Args:
            params: The module's frame axes params.
        """
        self._params = params
        self._drawn: set[str] = set()

    def add(self, *frames: str) -> None:
        """Draw frames not drawn yet. Their axes then follow the frames as their transforms change.

        Args:
            frames: Frame names, e.g. a transform's parent and child frames.
        """
        if self._params.axis_length is None:
            return
        for frame in frames:
            if frame and frame not in self._drawn:
                rr.log(
                    f"frames/{rr.escape_entity_path_part(frame)}",
                    rr.CoordinateFrame(frame=frame),
                    rr.TransformAxes3D(axis_length=self._params.axis_length, show_frame=self._params.frame_names),
                    static=True,
                )
                self._drawn.add(frame)


class TFParams(FrameAxesParams):
    """Parameters for the TFModule."""

    static: bool = False
    """Log transforms as static, i.e. valid at all times. Set for /tf_static."""


@REGISTRY.register("tf")
class TFModule(TopicToComponentModule):
    """Module for forwarding tf2 transforms to Rerun as relations between named frames.

    Other modules then only name the frame their data is in (`header.frame_id`), as in rviz.

    Rerun requires a frame relation's changes over time to be logged on one entity; a static value on another entity
    is allowed, and acts as a default. Use one module for /tf and another, with `static`, for /tf_static, the latter
    on `tf_static`: the entity Rerun's URDF importer logs the robot's joint defaults on.
    """

    PARAMS = TFParams
    params: TFParams  # type hint for self.params

    def __init__(self, node: Node, config: ModuleConfig, context: BridgeContext) -> None:
        """Subscribe, and set up drawing the frames' axes.

        Args:
            node: The ROS node to use for subscriptions & logging.
            config: The module configuration.
            context: Shared context for all modules.
        """
        super().__init__(node, config, context)
        self._frame_axes = FrameAxes(self.params)

    @classmethod
    def ros_msg_type(cls):
        return TFMessage

    def handle(self, msg: TFMessage) -> None:
        # One row per transform: each has its own stamp
        for tf in msg.transforms:
            if not self.params.static:
                set_rr_time_from_ros(stamp_or_now(Time.from_msg(tf.header.stamp), self.node))
            t = tf.transform.translation
            q = tf.transform.rotation
            rr.log(
                self.entity_path,
                rr.Transform3D(
                    translation=[t.x, t.y, t.z],
                    quaternion=[q.x, q.y, q.z, q.w],
                    parent_frame=tf.header.frame_id,
                    child_frame=tf.child_frame_id,
                ),
                static=self.params.static,
            )
            self._frame_axes.add(tf.header.frame_id, tf.child_frame_id)
