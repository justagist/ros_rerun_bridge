from __future__ import annotations

import importlib
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Optional, Type

from .utils.qos import make_qos
from .utils.time_utils import set_rr_time_from_ros

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile

    from .context import BridgeContext


@dataclass
class ModuleSpec:
    name: str  # logical name for this subscription
    module: str  # registry key or python path to class
    topic: str  # ROS topic to subscribe to
    entity_path: str  # Rerun entity path for logging
    msg_type: Optional[str] = None  # python path to ROS msg class (overrides module default if provided)
    qos: dict = field(default_factory=dict)  # depth, reliability, durability, history
    extra: dict = field(default_factory=dict)


class TopicToComponentModule:
    """Base class for pluggable ROS→Rerun bridges for a single topic type.

    Subclasses should:
      * implement `ros_msg_type()` returning a ROS msg class (if msg_type not provided in config)
      * implement `handle(msg)` to log to Rerun using `self.entity_path`
      * optionally use `self.extra` for module-specific parameters
    """

    def __init__(self, node: Node, spec: ModuleSpec, context: BridgeContext) -> None:
        self.node = node
        self.context = context
        self.spec = spec
        self.entity_path = spec.entity_path
        self.extra = spec.extra or {}
        self._sub = None

        qos_profile: QoSProfile = make_qos(**spec.qos) if spec.qos else make_qos()

        msg_type = self._resolve_msg_type()
        self._sub = node.create_subscription(msg_type, spec.topic, self._callback, qos_profile)
        node.get_logger().info(f"[{spec.name}] subscribed to {spec.topic} with {msg_type.__name__} → {spec.entity_path}")

    # ——— Overridables ———
    @classmethod
    def ros_msg_type(cls):
        """Return the default ROS msg type (class). Subclasses must override unless config.msg_type is set."""
        raise NotImplementedError

    def handle(self, msg: Any) -> None:
        """Process & log to Rerun. Must be implemented by subclasses."""
        raise NotImplementedError

    # ——— Internals ———
    def _callback(self, msg: Any) -> None:
        stamp_getter: Optional[Callable[[Any], Optional["Time"]]] = getattr(self, "_extract_time", None)
        if stamp_getter:
            try:
                stamp = stamp_getter(msg)
                if stamp is not None:
                    set_rr_time_from_ros(stamp)
            except Exception as e:
                self.node.get_logger().warn(f"[{self.spec.name}] failed to set rr time: {e}")
        try:
            self.handle(msg)
        except Exception as e:
            self.node.get_logger().error(f"[{self.spec.name}] handle() error: {e}")

    def _resolve_msg_type(self) -> Type:
        if self.spec.msg_type:
            return _import_by_path(self.spec.msg_type)
        return self.ros_msg_type()


# Utility: import a dotted path "a.b.c:Class" or "a.b.c.Class"


def _import_by_path(path: str):
    if ":" in path:
        mod, attr = path.split(":", 1)
        return getattr(importlib.import_module(mod), attr)
    if "." in path:
        mod, attr = path.rsplit(".", 1)
        return getattr(importlib.import_module(mod), attr)
    raise ValueError(
        f"Invalid module path '{path}'. Expected 'pkg.mod:Class' or 'pkg.mod.Class'. "
        "If you intended a registry key (e.g. 'image'), ensure 'rerun_ros_bridge.modules' "
        "is imported so the key is registered."
    )
