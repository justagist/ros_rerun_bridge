import importlib

import numpy as np
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

import rerun as rr


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
        "If you intended a registry key (e.g. 'image'), ensure 'ros_rerun_bridge.modules' "
        "is imported so the key is registered."
    )


def set_rr_time_from_ros(stamp: Time) -> None:
    """Set Rerun global time from a ROS Time stamp."""
    rr.set_time("ros_time", timestamp=np.datetime64(stamp.nanoseconds, "ns"))


def make_qos(
    depth: int = 10,
    reliability: str | None = None,
    durability: str | None = None,
    history: str | None = None,
) -> QoSProfile:
    """Helper function to create a QoSProfile from given settings."""
    rel = {
        None: QoSReliabilityPolicy.RELIABLE,
        "reliable": QoSReliabilityPolicy.RELIABLE,
        "best_effort": QoSReliabilityPolicy.BEST_EFFORT,
    }[reliability]
    dur = {
        None: QoSDurabilityPolicy.VOLATILE,
        "volatile": QoSDurabilityPolicy.VOLATILE,
        "transient_local": QoSDurabilityPolicy.TRANSIENT_LOCAL,
    }[durability]
    hist = {
        None: QoSHistoryPolicy.KEEP_LAST,
        "keep_last": QoSHistoryPolicy.KEEP_LAST,
        "keep_all": QoSHistoryPolicy.KEEP_ALL,
    }[history]
    return QoSProfile(depth=depth, reliability=rel, durability=dur, history=hist)
