from __future__ import annotations

import os
import traceback
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, ValidationError
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import rerun as rr

# Imported at run time, not only for type checking, so that custom modules importing them from here keep working
from .config import ModuleConfig, QoSConfig
from .utils.utils import _import_module_by_path, make_qos, set_rr_time_from_ros, stamp_or_now

if TYPE_CHECKING:
    from rclpy.node import Node

    from .context import BridgeContext


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

    def _subscribe(self) -> None:
        """Subscribe to the topic. The builder calls it once the module is fully initialised: with automatic discovery,
        modules are built while the executor runs, and a message could otherwise reach a half-built module."""
        self._sub = self.node.create_subscription(
            self._msg_class, self._topic, self._callback, self._qos_profile, callback_group=self.callback_group
        )
        self.node.get_logger().info(
            f"[{self.name}] subscribed to {self._topic} with {self._msg_class.__name__} -> {self.entity_path}"
        )

    def close(self) -> None:
        """Unsubscribe, when automatic discovery drops the module's topic. A module holding other ROS resources, e.g. a
        timer, extends it to release them."""
        self.node.destroy_subscription(self._sub)

    def clear(self) -> None:
        """Clear what the module drew, when automatic discovery drops its topic, at the current Rerun time. Only the
        module's own entity: a recursive clear would also clear other modules' entities beneath it. A module drawing
        beneath its entity extends it to clear those too, and one whose drawing stays true, e.g. history, overrides it
        to keep it."""
        rr.log(self.entity_path, rr.Clear(recursive=False))

    def _resubscribe(self, qos: QoSConfig) -> None:
        """Subscribe again with another QoS, keeping the module's state, e.g. markers to delete later.

        Args:
            qos: The new subscription QoS.
        """
        self.node.destroy_subscription(self._sub)
        self._qos_profile = make_qos(qos)
        self._subscribe()

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
