from __future__ import annotations

from typing import Literal

from rcl_interfaces.msg import Log
from rclpy.time import Time

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import StrictParamsModel

_LEVELS = {"debug": Log.DEBUG, "info": Log.INFO, "warn": Log.WARN, "error": Log.ERROR, "fatal": Log.FATAL}
# Rerun has no FATAL: CRITICAL is its most severe level
_RERUN_LEVELS = {Log.DEBUG: "DEBUG", Log.INFO: "INFO", Log.WARN: "WARN", Log.ERROR: "ERROR", Log.FATAL: "CRITICAL"}


class RosoutParams(StrictParamsModel):
    """Parameters for the RosoutModule."""

    min_level: Literal["debug", "info", "warn", "error", "fatal"] = "debug"
    """Least severe level shown."""


def named_level(level: int) -> int:
    """A ROS log level as one of ROS's named levels: the named one at or below it, or DEBUG below them all."""
    return max((named for named in _RERUN_LEVELS if named <= level), default=Log.DEBUG)


@REGISTRY.register("rosout")
class RosoutModule(TopicToComponentModule):
    """Module for showing ROS log entries, from rcl_interfaces/Log on /rosout, in Rerun's text log view.

    Each logger's entries are logged on `<entity_path>/<logger name>`, with the name's dots as slashes, so the viewer
    can show or hide each node's logs.
    """

    PARAMS = RosoutParams
    params: RosoutParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):
        return Log

    def _extract_time(self, msg: Log) -> Time | None:
        return Time.from_msg(msg.stamp)

    def handle(self, msg: Log) -> None:
        level = named_level(msg.level)
        if level < _LEVELS[self.params.min_level]:
            return
        logger = "/".join(rr.escape_entity_path_part(part) for part in msg.name.split("."))
        rr.log(f"{self.entity_path}/{logger}", rr.TextLog(msg.msg, level=_RERUN_LEVELS[level]))
