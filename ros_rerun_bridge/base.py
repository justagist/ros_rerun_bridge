from __future__ import annotations

import os
import traceback
from fnmatch import fnmatchcase
from typing import TYPE_CHECKING, Any, Literal

from pydantic import BaseModel, Field, PositiveInt, ValidationError, field_validator
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import rerun as rr

from .types import StrictParamsModel
from .utils.frame_axes import FrameAxesParams
from .utils.utils import _import_module_by_path, make_qos, set_rr_time_from_ros, stamp_or_now

if TYPE_CHECKING:
    from rclpy.node import Node

    from .context import BridgeContext


class QoSConfig(StrictParamsModel):
    """Subscription QoS for a module. The publisher's QoS must be compatible, e.g. `best_effort` for sensor data."""

    depth: PositiveInt = 10
    reliability: Literal["reliable", "best_effort"] = "reliable"
    durability: Literal["volatile", "transient_local"] = "volatile"
    history: Literal["keep_last", "keep_all"] = "keep_last"


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
    qos: QoSConfig = Field(default_factory=QoSConfig)
    """Quality of Service settings."""
    params: dict = Field(default_factory=dict)
    """Additional parameters for the module."""


class AutoFrameAxesParams(FrameAxesParams):
    """How discovered TF frames are drawn: the `tf` module's frame params, with axes by default."""

    axis_length: float | None = Field(default=0.1, gt=0, allow_inf_nan=False)
    """Draw each frame's axes, this long in metres, or none if null. Names are off by default, as many frames would
    crowd the view, and the viewer can show each frame's name."""


class AutoConfig(StrictParamsModel):
    """Settings for automatic discovery: which topics it may bridge, as `fnmatch` globs on the topic name (`*` also
    matches `/`), and how it draws TF frames."""

    include: list[str] = Field(default_factory=lambda: ["*"])
    """Topics to bridge, if they have a module for their type."""
    exclude: list[str] = Field(default_factory=list)
    """Topics never to bridge, even if included."""
    frames: AutoFrameAxesParams = Field(default_factory=AutoFrameAxesParams)
    """How the frames from /tf and /tf_static are drawn."""

    def matches(self, topic: str) -> bool:
        """Whether the topic is included and not excluded."""
        return any(fnmatchcase(topic, p) for p in self.include) and not any(fnmatchcase(topic, p) for p in self.exclude)


class BridgeConfig(StrictParamsModel):
    """Configuration for the ROS->Rerun bridge."""

    global_frame: str = "map"
    """TF frame that Rerun's default 3D view is rendered in."""
    blueprint: str | None = None
    """Viewer layout to open with: a .rbl file, e.g. saved from the viewer. Relative paths are to the config file."""
    auto: AutoConfig | None = None
    """Automatic discovery of the topics `modules` does not name: `true`, or the topics it may bridge. Off if unset."""
    modules: list[ModuleConfig] = Field(default_factory=list)
    """List of bridge modules."""

    @field_validator("auto", mode="before")
    @classmethod
    def _auto_from_bool(cls, value: Any) -> Any:
        """Accept `auto: true` for discovery with the default filters, and `false` for none."""
        if isinstance(value, bool):
            return AutoConfig() if value else None
        return value


class TopicToComponentModule:
    """Base class for modules, each logging one ROS topic to Rerun. See docs/EXTENDING.md.

    Subclasses implement `ros_msg_type()` (unless the config sets `msg_type`) and `handle(msg)`, and may set `PARAMS`
    to a params model and implement `_extract_time(msg)`.
    """

    PARAMS: type[BaseModel] | None = None
    """Model the config's `params` are validated into, as `self.params`. Without one, `self.params` is a dict."""

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
                raise ValueError(f"invalid 'params' config for {self.__class__.__name__}: {e}") from e

        self._sub = None
        self._topic = config.topic
        self._qos_profile = make_qos(config.qos)
        self._msg_class = self._resolve_msg_type()
        # One group per module: topics are handled in parallel, but each module sees its messages one at a time.
        # Modules add their own timers to it to keep that guarantee.
        self.callback_group = MutuallyExclusiveCallbackGroup()

    def start(self) -> None:
        """Subscribe to the topic. The builder calls it once the module is fully initialised: with automatic discovery,
        modules are built while the executor runs, and a message could otherwise reach a half-built module."""
        self._sub = self.node.create_subscription(
            self._msg_class, self._topic, self._callback, self._qos_profile, callback_group=self.callback_group
        )
        self.node.get_logger().info(
            f"[{self.name}] subscribed to {self._topic} with {self._msg_class.__name__} -> {self.entity_path}"
        )

    # Overridables
    @classmethod
    def ros_msg_type(cls):
        """Return the default ROS msg type (class). Subclasses must override unless config.msg_type is set."""
        raise NotImplementedError

    def handle(self, msg: Any) -> None:
        """Process & log to Rerun. This method is called when a new message is received. Subclasses must override."""
        raise NotImplementedError

    # Helpers
    def log(self, *archetypes: Any, frame_id: str = "") -> None:
        """Log archetypes to this module's entity, placed in the named TF frame `frame_id` if one is given.

        Args:
            archetypes: Rerun archetypes to log.
            frame_id: TF frame the data is expressed in, usually the message's `header.frame_id`.
        """
        if frame_id:
            archetypes = (*archetypes, rr.CoordinateFrame(frame=frame_id))
        rr.log(self.entity_path, *archetypes)

    # Internals
    def _callback(self, msg: Any) -> None:
        # An exception would stop the executor, and with it every module
        try:
            # Rerun's time is per thread and executor threads are shared between modules, so it must be set for every
            # message
            stamp_getter = getattr(self, "_extract_time", None)
            set_rr_time_from_ros(stamp_or_now(stamp_getter(msg) if stamp_getter else None, self.node))
            self.handle(msg)
        except Exception as e:
            logger = self.node.get_logger()
            # Where it was raised, as the full traceback is only logged at debug level
            frame = traceback.extract_tb(e.__traceback__)[-1]
            where = f"{frame.name} ({os.path.basename(frame.filename)}:{frame.lineno})"
            logger.error(
                f"[{self.name}] error handling a message: {type(e).__name__}: {e}, in {where}",
                throttle_duration_sec=5.0,
            )
            logger.debug(traceback.format_exc())

    def _resolve_msg_type(self) -> type:
        if self.msg_type:
            return _import_module_by_path(self.msg_type)
        return self.ros_msg_type()
