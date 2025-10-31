from __future__ import annotations

from typing import TYPE_CHECKING

from rclpy.time import Duration, Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rerun as rr

if TYPE_CHECKING:
    from rclpy.node import Node


class TFHelper:
    def __init__(self, node: Node) -> None:
        self.buffer = Buffer()

        self.listener = TransformListener(self.buffer, node)
        self.node = node

    def log_tf_path(self, path: str, child_frame: str, parent_frame: str, time: Time, *, timeout_sec: float = 0.1) -> None:
        try:
            tf = self.buffer.lookup_transform(parent_frame, child_frame, time, timeout=Duration(seconds=timeout_sec))
            t = tf.transform.translation
            q = tf.transform.rotation
            rr.log(path, rr.Transform3D(translation=[t.x, t.y, t.z], rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w])))
        except TransformException as ex:
            self.node.get_logger().warn(f"TF lookup failed {parent_frame}←{child_frame}: {ex}")
