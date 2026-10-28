from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from rclpy.time import Time

import rerun as rr

from ..base import ModuleConfig, TopicToComponentModule
from ..context import BridgeContext
from ..registry import REGISTRY
from ..types import StrictParamsModel
from ..utils.utils import stamp_or_now

# Label, colour and Rerun log level per diagnostic level, which ROS gives as a byte
_LEVELS = {
    ord(DiagnosticStatus.OK): ("OK", (40, 170, 70, 255), "INFO"),
    ord(DiagnosticStatus.WARN): ("WARN", (240, 180, 0, 255), "WARN"),
    ord(DiagnosticStatus.ERROR): ("ERROR", (220, 40, 40, 255), "ERROR"),
    ord(DiagnosticStatus.STALE): ("STALE", (130, 130, 130, 255), "WARN"),
}


@REGISTRY.register("diagnostics")
class DiagnosticsModule(TopicToComponentModule):
    """Module for showing diagnostic_msgs/DiagnosticArray statuses, e.g. from /diagnostics or /diagnostics_agg.

    Each status is logged on `<entity_path>/<status name>`, with the name split on slashes, so an aggregator's
    `/Robot/Arm/Motor` nests as a tree. Its level is shown as a state, in Rerun's state timeline view, and its message
    in the text log view, each logged only when it changes.
    """

    # No params, so any given are an error
    PARAMS = StrictParamsModel

    @classmethod
    def ros_msg_type(cls):
        return DiagnosticArray

    def __init__(self, node: Node, config: ModuleConfig, context: BridgeContext) -> None:
        """Subscribe.

        Args:
            node: The ROS node to use for subscriptions & logging.
            config: The module configuration.
            context: Shared context for all modules.
        """
        super().__init__(node, config, context)
        self._latest: dict[str, tuple[int, str, int]] = {}
        """Level, message and time, in ns, of the latest status in time, per entity path."""

    def _extract_time(self, msg: DiagnosticArray) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: DiagnosticArray) -> None:
        stamp = stamp_or_now(self._extract_time(msg), self.node).nanoseconds
        for status in msg.status:
            parts = [rr.escape_entity_path_part(part) for part in status.name.split("/") if part]
            path = "/".join([self.entity_path, *parts])
            level = ord(status.level)
            latest = self._latest.get(path)
            if latest is None:
                labels, colours, _ = zip(*_LEVELS.values())
                rr.log(path, rr.StateConfiguration(values=labels, colors=colours), static=True)
            label, _, log_level = _LEVELS.get(level, (str(level), None, "WARN"))
            # Earlier than the latest status, e.g. after a bag is replayed: logged as it is, as it is not a change from
            # the latest
            earlier = latest is not None and stamp < latest[2]
            if earlier or latest is None or latest[0] != level:
                rr.log(path, rr.StateChange(state=label))
            if earlier or latest is None or latest[:2] != (level, status.message):
                rr.log(path, rr.TextLog(status.message, level=log_level))
            if not earlier:
                self._latest[path] = (level, status.message, stamp)
