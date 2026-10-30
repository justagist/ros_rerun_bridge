from __future__ import annotations

from rclpy.node import Node
from rclpy.time import Time
from vision_msgs.msg import Detection3DArray

import rerun as rr

from ..base import ModuleConfig, TopicToComponentModule
from ..context import BridgeContext
from ..registry import REGISTRY
from ..utils.detections import DetectionClasses, DetectionParams
from .pose import quaternion_xyzw


@REGISTRY.register("detections_3d")
class Detections3DModule(TopicToComponentModule):
    """Module for drawing vision_msgs/Detection3DArray boxes, labelled with their class, tracking id and score."""

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
        return Detection3DArray

    def _extract_time(self, msg: Detection3DArray) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Detection3DArray) -> None:
        if not msg.detections:
            self.log(rr.Boxes3D.cleared(), frame_id=msg.header.frame_id)
            return
        boxes = [detection.bbox for detection in msg.detections]
        class_ids, labels = self._classes.class_ids_and_labels(msg.detections)
        rr_boxes = rr.Boxes3D(
            centers=[[box.center.position.x, box.center.position.y, box.center.position.z] for box in boxes],
            sizes=[[box.size.x, box.size.y, box.size.z] for box in boxes],
            quaternions=[rr.Quaternion(xyzw=quaternion_xyzw(box.center.orientation)) for box in boxes],
            class_ids=class_ids,
            labels=labels,
        )
        self.log(rr_boxes, frame_id=msg.header.frame_id)
