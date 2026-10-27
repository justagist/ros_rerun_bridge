from __future__ import annotations

import importlib
from typing import TYPE_CHECKING

import numpy as np
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

import rerun as rr

if TYPE_CHECKING:
    from rclpy.node import Node

    from ..base import QoSConfig

IMAGE_PLANE_SUFFIX = "_image_plane"
"""Appended to a camera's frame_id to name the frame its images are in, linked to the camera frame by its pinhole."""


def _import_module_by_path(path: str):
    # Utility: import a dotted path "a.b.c:Class" or "a.b.c.Class"
    if ":" in path:
        mod, attr = path.split(":", 1)
        return getattr(importlib.import_module(mod), attr)
    if "." in path:
        mod, attr = path.rsplit(".", 1)
        return getattr(importlib.import_module(mod), attr)
    raise ValueError(
        f"Invalid module path '{path}'. Expected 'pkg.mod:Class' or 'pkg.mod.Class'. "
        "If you intended a built-in module (e.g. 'image'), check its name."
    )


def set_rr_time_from_ros(stamp: Time) -> None:
    """Set Rerun global time from a ROS Time stamp."""
    rr.set_time("ros_time", timestamp=np.datetime64(stamp.nanoseconds, "ns"))


def make_qos(qos: QoSConfig) -> QoSProfile:
    """Create a QoSProfile from a module's validated QoS settings."""
    return QoSProfile(
        depth=qos.depth,
        reliability=QoSReliabilityPolicy[qos.reliability.upper()],
        durability=QoSDurabilityPolicy[qos.durability.upper()],
        history=QoSHistoryPolicy[qos.history.upper()],
    )


def stamp_or_now(stamp: Time | None, node: Node) -> Time:
    """Return a message's stamp, or the node's current time for a message with none (or a zero stamp)."""
    if stamp is None or stamp.nanoseconds == 0:
        return node.get_clock().now()
    return stamp
