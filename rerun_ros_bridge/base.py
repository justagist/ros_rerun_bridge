from __future__ import annotations

import importlib
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Optional, Type

from pydantic import BaseModel, Field, ValidationError

from .types import StrictParamsModel
from .utils.qos import make_qos
from .utils.time_utils import set_rr_time_from_ros

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile

    from .context import BridgeContext


class ModuleConfig(StrictParamsModel):
    name: str  # logical name for this subscription
    module: str  # registry key or python path to class
    topic: str  # ROS topic to subscribe to
    entity_path: str  # Rerun entity path for logging
    msg_type: str | None = None  # python path to ROS msg class (overrides module default if provided)
    qos: dict = Field(default_factory=dict)  # depth, reliability, durability, history
    params: dict = Field(default_factory=dict)


class BridgeConfig(StrictParamsModel):
    global_frame: str = "map"
    modules: list[ModuleConfig]


class TopicToComponentModule:
    """Base class for pluggable ROS→Rerun bridges for a single topic type.

    Subclasses should:
      * implement `ros_msg_type()` returning a ROS msg class (if msg_type not provided in config)
      * implement `handle(msg)` to log to Rerun using `self.entity_path`

    Subclasses MAY define `PARAMS` = <PydanticModelSubclass>.
    If present, `self.params` will be an instance of that type (validated),
    otherwise it remains a plain dict.
    """

    PARAMS: Optional[BaseModel] = None  # subclasses can override

    def __init__(self, node: Node, config: ModuleConfig, context: BridgeContext) -> None:
        self.node = node
        self.context = context
        self.name = config.name
        self.msg_type = config.msg_type
        self.entity_path = config.entity_path
        self.params = config.params.copy() or {}
        if self.PARAMS is not None:
            try:
                self.params = self.PARAMS.model_validate(self.params)
            except ValidationError as e:
                raise ValueError(f"[{config.name}] invalid 'params' config for {self.__class__.__name__}: {e}") from e

        self._sub = None

        qos_profile: QoSProfile = make_qos(**config.qos) if config.qos else make_qos()

        msg_type = self._resolve_msg_type()
        self._sub = node.create_subscription(msg_type, config.topic, self._callback, qos_profile)
        node.get_logger().info(
            f"[{config.name}] subscribed to {config.topic} with {msg_type.__name__} → {config.entity_path}"
        )

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
                self.node.get_logger().warn(f"[{self.name}] failed to set rr time: {e}")
        try:
            self.handle(msg)
        except Exception as e:
            self.node.get_logger().error(f"[{self.name}] handle() error: {e}")

    def _resolve_msg_type(self) -> Type:
        if self.msg_type:
            return _import_module_by_path(self.msg_type)
        return self.ros_msg_type()


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
        "If you intended a registry key (e.g. 'image'), ensure 'rerun_ros_bridge.modules' "
        "is imported so the key is registered."
    )
