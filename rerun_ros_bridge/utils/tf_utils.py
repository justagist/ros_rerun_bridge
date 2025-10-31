from __future__ import annotations

from typing import TYPE_CHECKING

from rclpy.time import Duration, Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rerun as rr

if TYPE_CHECKING:
    from rclpy.node import Node

    from ..types import TFPath


class TFHelper:
    def __init__(self, node: Node) -> None:
        self.buffer = Buffer()

        self.listener = TransformListener(self.buffer, node)
        self.node = node

    def log_tf_path(self, tf_path: TFPath, time: Time, *, timeout_sec: float = 0.1) -> None:
        try:
            tf = self.buffer.lookup_transform(
                tf_path.parent_frame, tf_path.child_frame, time, timeout=Duration(seconds=timeout_sec)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            rr.log(
                tf_path.path, rr.Transform3D(translation=[t.x, t.y, t.z], rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]))
            )
        except TransformException as ex:
            self.node.get_logger().warn(f"TF lookup failed {tf_path.parent_frame}←{tf_path.child_frame}: {ex}")
