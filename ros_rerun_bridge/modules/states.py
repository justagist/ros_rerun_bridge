from __future__ import annotations

from operator import attrgetter
from typing import Any

from pydantic import model_validator
from rclpy.node import Node
from rclpy.time import Time

import rerun as rr

from ..base import ModuleConfig, TopicToComponentModule
from ..context import BridgeContext
from ..registry import REGISTRY
from ..types import Rgba, StrictParamsModel
from ..utils.fields import field_type, is_single_value


class StateStyle(StrictParamsModel):
    """How one state value is shown."""

    value: bool | int | float | str
    """The field's value, e.g. `1` or `"running"`."""
    label: str | None = None
    """Name shown for the state. The value itself if unset."""
    colour: Rgba | None = None
    """RGBA colour of the state, as uint8. Give one for every state, or for none to let Rerun pick them."""


class StatesParams(StrictParamsModel):
    """Parameters for the StatesModule."""

    field: str
    """Dotted path to a single number, bool or string, e.g. `data` or `brightness`."""
    states: list[StateStyle] = []
    """Label and colour per value. Values not listed are shown as they are."""

    @model_validator(mode="after")
    def _check_colours(self) -> StatesParams:
        """Rerun takes colours as a list parallel to the states, so it cannot leave one out in the middle."""
        coloured = [s.colour is not None for s in self.states]
        if any(coloured) and not all(coloured):
            raise ValueError("give a colour for every state, or for none")
        return self


def state_text(value: bool | int | float | str) -> str:
    """A field or config value as a state: `1`, `1.0` and a float32 1.0 all give "1", and booleans are lowercase."""
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, float):
        return f"{value:g}"
    return str(value)


@REGISTRY.register("states")
class StatesModule(TopicToComponentModule):
    """Module for showing a field of any message type as discrete states over time, e.g. modes or lights on and off.

    Each change of the field's value starts a new state, shown in Rerun's state timeline view. The config must set
    `msg_type`.
    """

    PARAMS = StatesParams
    params: StatesParams  # type hint for self.params

    def __init__(self, node: Node, config: ModuleConfig, context: BridgeContext) -> None:
        """Subscribe, check the configured field, and log the state labels and colours.

        Args:
            node: The ROS node to use for subscriptions & logging.
            config: The module configuration.
            context: Shared context for all modules.
        """
        if not config.msg_type:
            raise ValueError(f"[{config.name}] the states module needs `msg_type` set")
        super().__init__(node, config, context)
        ros_type = field_type(self._resolve_msg_type(), self.params.field)
        if not is_single_value(ros_type):
            raise ValueError(f"field '{self.params.field}' is {ros_type}, not a number, bool or string")
        self._get = attrgetter(self.params.field)
        self._state: str | None = None
        """The state last logged, as each change marks a new one."""
        if self.params.states:
            states = self.params.states
            rr.log(
                self.entity_path,
                rr.StateConfiguration(
                    values=[state_text(s.value) for s in states],
                    labels=[s.label if s.label is not None else state_text(s.value) for s in states],
                    colors=[s.colour for s in states] if states[0].colour is not None else None,
                ),
                static=True,
            )

    def _extract_time(self, msg: Any) -> Time | None:
        header = getattr(msg, "header", None)
        return Time.from_msg(header.stamp) if header is not None else None

    def handle(self, msg: Any) -> None:
        state = state_text(self._get(msg))
        if state != self._state:
            rr.log(self.entity_path, rr.StateChange(state=state))
            self._state = state
