from __future__ import annotations

from operator import attrgetter
from typing import Any

import numpy as np
from rclpy.node import Node
from rclpy.time import Time

import rerun as rr

from ..base import ModuleConfig, TopicToComponentModule
from ..context import BridgeContext
from ..registry import REGISTRY
from ..types import Rgba, StrictParamsModel
from ..utils.fields import check_field, is_array


class BarChartParams(StrictParamsModel):
    """Parameters for the BarChartModule."""

    field: str
    """Dotted path to the array of numbers to draw as bars, e.g. `effort`."""
    colour: Rgba | None = None
    """RGBA colour of the bars, as uint8. Rerun's default if unset."""


@REGISTRY.register("bar_chart")
class BarChartModule(TopicToComponentModule):
    """Module for drawing an array field of any message type as a bar chart, one bar per element.

    The chart shows the array at the viewer's time cursor, so scrubbing the timeline replays it. The config must set
    `msg_type`.
    """

    PARAMS = BarChartParams
    params: BarChartParams  # type hint for self.params

    def __init__(self, node: Node, config: ModuleConfig, context: BridgeContext) -> None:
        """Subscribe, and check the configured field against the message type.

        Args:
            node: The ROS node to use for subscriptions & logging.
            config: The module configuration.
            context: Shared context for all modules.
        """
        if not config.msg_type:
            raise ValueError("the bar_chart module needs `msg_type` set")
        super().__init__(node, config, context)
        field_type = check_field(self._resolve_msg_type(), self.params.field)
        if not is_array(field_type):
            raise ValueError(f"field '{self.params.field}' is {field_type}, not an array")
        self._get = attrgetter(self.params.field)

    def _extract_time(self, msg: Any) -> Time | None:
        header = getattr(msg, "header", None)
        return Time.from_msg(header.stamp) if header is not None else None

    def handle(self, msg: Any) -> None:
        rr.log(self.entity_path, rr.BarChart(np.asarray(self._get(msg), dtype=np.float64), color=self.params.colour))
