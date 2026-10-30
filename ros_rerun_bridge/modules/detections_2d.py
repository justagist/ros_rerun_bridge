from __future__ import annotations

import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from vision_msgs.msg import Detection2DArray

import rerun as rr

from ..base import ModuleConfig, TopicToComponentModule
from ..context import BridgeContext
from ..registry import REGISTRY
from ..utils.detections import DetectionClasses, DetectionParams
from ..utils.utils import IMAGE_PLANE_SUFFIX

# A box's corners, from its top left, going round to the top left again to close it. Rerun labels a line strip at its
# points' mean, so with the top left twice, labels lean towards it.
_CORNERS = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]])


@REGISTRY.register("detections_2d")
class Detections2DModule(TopicToComponentModule):
    """Module for drawing vision_msgs/Detection2DArray boxes, labelled with their class, tracking id and score.

    Boxes are in pixels, so they are logged on the camera's image plane like images, and drawn over the camera image in
    2D and in 3D. They are drawn as closed line strips, as Rerun's 2D boxes cannot be rotated.
    """

    PARAMS = DetectionParams
    params: DetectionParams  # type hint for self.params

    def __init__(self, node: Node, config: ModuleConfig, context: BridgeContext) -> None:
        """Subscribe, and log the class colours.

        Args:
            node: The ROS node to use for subscriptions & logging.
            config: The module configuration.
            context: Shared context for all modules.
        """
        super().__init__(node, config, context)
        self._classes = DetectionClasses(self.entity_path, self.params.classes)

    @classmethod
    def ros_msg_type(cls):
        return Detection2DArray

    def _extract_time(self, msg: Detection2DArray) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Detection2DArray) -> None:
        frame = msg.header.frame_id
        frame_id = f"{frame}{IMAGE_PLANE_SUFFIX}" if frame else ""
        if not msg.detections:
            self.log(rr.LineStrips2D.cleared(), frame_id=frame_id)
            return
        boxes = [detection.bbox for detection in msg.detections]
        centres = np.array([[box.center.position.x, box.center.position.y] for box in boxes])
        half_sizes = np.array([[box.size_x, box.size_y] for box in boxes]) / 2
        theta = np.array([box.center.theta for box in boxes])[:, None]
        offsets = _CORNERS * half_sizes[:, None]
        x, y = offsets[..., 0], offsets[..., 1]
        rotated = np.stack([np.cos(theta) * x - np.sin(theta) * y, np.sin(theta) * x + np.cos(theta) * y], axis=-1)
        class_ids, labels = self._classes.class_ids_and_labels(msg.detections)
        strips = rr.LineStrips2D(centres[:, None] + rotated, class_ids=class_ids, labels=labels)
        self.log(strips, frame_id=frame_id)
