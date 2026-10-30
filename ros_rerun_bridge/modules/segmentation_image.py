from __future__ import annotations

from typing import Annotated

from pydantic import Field, model_validator
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image

import rerun as rr

from ..base import ModuleConfig, TopicToComponentModule
from ..context import BridgeContext
from ..registry import REGISTRY
from ..types import Rgba, StrictParamsModel
from ..utils.utils import IMAGE_PLANE_SUFFIX
from .image import pixels_from_msg

# Rerun's class ids are 16-bit
_CLASS_ID_ENCODINGS = ("mono8", "8UC1", "mono16", "16UC1")


class SegmentationClass(StrictParamsModel):
    """How one class is shown."""

    id: Annotated[int, Field(ge=0, le=65535)]
    """The class's pixel value."""
    label: str | None = None
    """Name shown for the class when hovering over it. Its id if unset."""
    colour: Rgba | None = None
    """RGBA colour of the class, as uint8. One Rerun picks for its id if unset."""


class SegmentationParams(StrictParamsModel):
    """Parameters for the SegmentationImageModule."""

    classes: list[SegmentationClass] = []
    """Label and colour per class id. Ids not listed are shown in a colour Rerun picks for each."""

    @model_validator(mode="after")
    def _check_ids(self) -> SegmentationParams:
        """A class listed twice would leave which of its labels and colours is shown up to Rerun."""
        ids = [c.id for c in self.classes]
        if len(set(ids)) != len(ids):
            raise ValueError("each class id can only be listed once")
        return self


@REGISTRY.register("segmentation_image")
class SegmentationImageModule(TopicToComponentModule):
    """Module for logging segmentation masks, images of class ids, as Rerun segmentation images.

    Masks are logged on the camera's image plane like images, so they are drawn over the camera image in 2D and in 3D.
    """

    PARAMS = SegmentationParams
    params: SegmentationParams  # type hint for self.params

    def __init__(self, node: Node, config: ModuleConfig, context: BridgeContext) -> None:
        """Subscribe, and log the class labels and colours.

        Args:
            node: The ROS node to use for subscriptions & logging.
            config: The module configuration.
            context: Shared context for all modules.
        """
        super().__init__(node, config, context)
        if self.params.classes:
            classes = [rr.AnnotationInfo(id=c.id, label=c.label, color=c.colour) for c in self.params.classes]
            rr.log(self.entity_path, rr.AnnotationContext(classes), static=True)

    @classmethod
    def ros_msg_type(cls):
        return Image

    def _extract_time(self, msg: Image) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Image) -> None:
        if msg.encoding not in _CLASS_ID_ENCODINGS:
            raise ValueError(f"'{msg.encoding}' is not a class id encoding: {', '.join(_CLASS_ID_ENCODINGS)}")
        frame = msg.header.frame_id
        self.log(rr.SegmentationImage(pixels_from_msg(msg)), frame_id=f"{frame}{IMAGE_PLANE_SUFFIX}" if frame else "")
