from __future__ import annotations

from typing import TYPE_CHECKING, Any, Callable, Optional, Type

from pydantic import BaseModel, Field, ValidationError
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from .types import StrictParamsModel
from .utils.utils import _import_module_by_path, make_qos, set_rr_time_from_ros

if TYPE_CHECKING:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from rclpy.time import Time

    from .context import BridgeContext


class ModuleConfig(StrictParamsModel):
    """Configuration for a single bridge module."""

    name: str
    """Name of the module."""
    module: str
    """registry key or python path to the module class. e.g. 'image' or 'my_pkg.my_mod:MyModule'"""
    topic: str
    """ROS topic to subscribe to."""
    entity_path: str
    """Rerun entity path for logging."""
    msg_type: str | None = None
    """Python path to ROS msg class (overrides module default if provided)."""
    qos: dict = Field(default_factory=dict)
    """Quality of Service settings (depth, reliability, durability, history)."""
    params: dict = Field(default_factory=dict)
    """Additional parameters for the module."""


class BridgeConfig(StrictParamsModel):
    """Configuration for the ROS→Rerun bridge."""

    global_frame: str = "map"
    """Global frame for TF logging."""
    modules: list[ModuleConfig]
    """List of bridge modules."""


class TopicToComponentModule:
    """Base class for pluggable ROS→Rerun bridges for a single topic type.

    Subclasses should:
      * implement `ros_msg_type()` returning a ROS msg class (if msg_type not provided in config)
      * implement `handle(msg)` to log to Rerun using `self.entity_path`

    Subclasses MAY define `PARAMS` = <PydanticModelSubclass>.
    If present, `self.params` will be an instance of that type (validated),
    otherwise it remains a plain dict.

    Subclasses MAY implement `_extract_time(msg) -> Optional[Time]` to extract
    a ROS time stamp from the message. The bridge sets Rerun's time to that
    stamp before calling `handle()`, or to the receive time if there is none.
    """

    PARAMS: Optional[BaseModel] = None  # subclasses can override

    def __init__(self, node: Node, config: ModuleConfig, context: BridgeContext) -> None:
        """Initialize the module.

        Args:
            node: The ROS node to use for subscriptions & logging.
            config: The module configuration.
            context: Shared context for all modules.
        """
        self.node = node
        """The ROS node to use for subscriptions & logging."""
        self.context = context
        """Shared context for all modules."""
        self.name = config.name
        """Name of the module."""
        self.msg_type = config.msg_type
        """Python path to ROS msg class (overrides module default if provided)."""
        self.entity_path = config.entity_path
        """Rerun entity path for logging."""
        self.params = config.params.copy() or {}
        """Additional parameters specific to the module."""
        if self.PARAMS is not None:
            try:
                self.params = self.PARAMS.model_validate(self.params)
            except ValidationError as e:
                raise ValueError(f"[{config.name}] invalid 'params' config for {self.__class__.__name__}: {e}") from e

        self._sub = None

        qos_profile: QoSProfile = make_qos(**config.qos) if config.qos else make_qos()

        msg_type = self._resolve_msg_type()
        # One group per module: topics are handled in parallel, but each module sees its messages one at a time
        self._sub = node.create_subscription(
            msg_type, config.topic, self._callback, qos_profile, callback_group=MutuallyExclusiveCallbackGroup()
        )
        node.get_logger().info(
            f"[{config.name}] subscribed to {config.topic} with {msg_type.__name__} → {config.entity_path}"
        )

    # ——— Overridables ———
    @classmethod
    def ros_msg_type(cls):
        """Return the default ROS msg type (class). Subclasses must override unless config.msg_type is set."""
        raise NotImplementedError

    def handle(self, msg: Any) -> None:
        """Process & log to Rerun. This method is called when a new message is received. Subclasses must override."""
        raise NotImplementedError

    # ——— Internals ———
    def _callback(self, msg: Any) -> None:
        # Rerun's time is per thread and executor threads are shared between modules, so it must be set for every
        # message. Unstamped messages (or a zero stamp) fall back to receive time.
        stamp_getter: Optional[Callable[[Any], Optional["Time"]]] = getattr(self, "_extract_time", None)
        stamp = stamp_getter(msg) if stamp_getter else None
        if stamp is None or stamp.nanoseconds == 0:
            stamp = self.node.get_clock().now()
        set_rr_time_from_ros(stamp)
        try:
            self.handle(msg)
        except Exception as e:
            self.node.get_logger().error(f"[{self.name}] handle() error: {e}", throttle_duration_sec=5.0)

    def _resolve_msg_type(self) -> Type:
        if self.msg_type:
            return _import_module_by_path(self.msg_type)
        return self.ros_msg_type()
