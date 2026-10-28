from __future__ import annotations

from operator import attrgetter
from typing import Any

from pydantic import model_validator
import numpy as np
from rclpy.node import Node
from rclpy.time import Time

import rerun as rr

from ..base import ModuleConfig, TopicToComponentModule
from ..context import BridgeContext
from ..registry import REGISTRY
from ..types import Rgba, StrictParamsModel
from ..utils.fields import field_type, is_single_value, is_string
from ..utils.utils import stamp_or_now


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


def state_text(value: bool | int | float | str, ros_type: str) -> str:
    """A field's value, or a config value for that field, as a state.

    Values that are equal in the field's type give the same text: `1`, `1.0` and a float32 1.0 are all "1", and a
    float32 0.3 is "0.3". Booleans are "true" or "false".

    Args:
        value: The value, from a message or from the config.
        ros_type: The field's ROS type, e.g. `float` or `string`.

    Raises:
        ValueError: If the field could never hold the value.
    """
    if ros_type == "boolean":
        if not isinstance(value, bool):
            raise ValueError(f"state value {value!r} is not a bool")
        return str(value).lower()
    if is_string(ros_type):
        if isinstance(value, bool):
            raise ValueError(
                f"state value {value!r} is not a string: quote it, as YAML reads on, off, yes and no as bools"
            )
        return str(value)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"state value {value!r} is not a number")
    if ros_type in ("float", "double"):
        number = np.float32(value) if ros_type == "float" else np.float64(value)
        # The shortest digits that give back the same number at the field's precision, never in exponent notation
        return np.format_float_positional(number, unique=True, trim="-")
    if not float(value).is_integer():
        raise ValueError(f"state value {value!r} is not an integer")
    return str(int(value))


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
            raise ValueError("the states module needs `msg_type` set")
        super().__init__(node, config, context)
        self._ros_type = field_type(self._resolve_msg_type(), self.params.field)
        if not is_single_value(self._ros_type):
            raise ValueError(f"field '{self.params.field}' is {self._ros_type}, not a number, bool or string")
        self._get = attrgetter(self.params.field)
        self._state: str | None = None
        """The latest state in time, as each change marks a new one."""
        self._stamp: int | None = None
        """The latest state's time, in ns."""
        if self.params.states:
            states = self.params.states
            values = [state_text(s.value, self._ros_type) for s in states]
            rr.log(
                self.entity_path,
                rr.StateConfiguration(
                    values=values,
                    labels=[s.label if s.label is not None else value for s, value in zip(states, values)],
                    colors=[s.colour for s in states] if states[0].colour is not None else None,
                ),
                static=True,
            )

    def _extract_time(self, msg: Any) -> Time | None:
        header = getattr(msg, "header", None)
        return Time.from_msg(header.stamp) if header is not None else None

    def handle(self, msg: Any) -> None:
        state = state_text(self._get(msg), self._ros_type)
        stamp = stamp_or_now(self._extract_time(msg), self.node).nanoseconds
        if self._stamp is not None and stamp < self._stamp:
            # Earlier than the latest state, e.g. after a bag is replayed: logged as it is, as it is not a change from
            # the latest
            rr.log(self.entity_path, rr.StateChange(state=state))
            return
        if state != self._state:
            rr.log(self.entity_path, rr.StateChange(state=state))
        self._state, self._stamp = state, stamp
