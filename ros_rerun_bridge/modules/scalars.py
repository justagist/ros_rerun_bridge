from __future__ import annotations

import re
from operator import attrgetter
from typing import Any, Literal

import numpy as np
from pydantic import Field, model_validator
from rclpy.node import Node
from rclpy.time import Time

import rerun as rr

from ..base import ModuleConfig, TopicToComponentModule
from ..context import BridgeContext
from ..registry import REGISTRY
from ..types import Rgba, StrictParamsModel

# Not octet (`byte`): its values are `bytes` objects, and numeric bytes are uint8 by convention
_NUMERIC = {
    "boolean",
    "float",
    "double",
    "int8",
    "uint8",
    "int16",
    "uint16",
    "int32",
    "uint32",
    "int64",
    "uint64",
}
# Element type of an unbounded or bounded sequence, e.g. "sequence<double, 3>", or of a fixed array, e.g. "double[9]"
_ARRAY = re.compile(r"sequence<(\w+)(?:, \d+)?>|(\w+)\[\d+\]")


class ScalarsParams(StrictParamsModel):
    """Parameters for the ScalarsModule."""

    fields: list[str] = Field(min_length=1)
    """Dotted paths to the numeric fields, or arrays of numbers, to plot, e.g. `range` or `twist.twist.linear.x`."""
    style: Literal["lines", "points"] = "lines"
    """Draw each series as a line or as points."""
    names: list[str] | None = None
    """Series names, one per array element, for every field. Rerun's default is the entity path and element index."""
    colours: list[Rgba] | None = None
    """RGBA colour per series, as uint8. The last one applies to any remaining series."""
    width: float | None = None
    """Line width, in UI points. Lines only."""
    interpolation: Literal["linear", "step_after", "step_before", "step_mid"] | None = None
    """How lines join values: linear, or steps, e.g. for discrete values. Lines only."""
    marker: (
        Literal["circle", "diamond", "square", "cross", "plus", "up", "down", "left", "right", "asterisk"] | None
    ) = None
    """Point marker shape. Points only."""
    marker_size: float | None = None
    """Point marker size, in UI points. Points only."""

    @model_validator(mode="after")
    def _check_style(self) -> ScalarsParams:
        """Reject options that only apply to the other style."""
        other = ("marker", "marker_size") if self.style == "lines" else ("width", "interpolation")
        if given := [name for name in other if getattr(self, name) is not None]:
            raise ValueError(f"{', '.join(given)} cannot be used with style '{self.style}'")
        return self


@REGISTRY.register("scalars")
class ScalarsModule(TopicToComponentModule):
    """Module for plotting numeric fields of any message type as time series.

    Each field is logged on `<entity_path>/<field>`, with the field's dots as slashes. An array is plotted as one series
    per element, with a gap in a series while an array is too short to have it. Non-finite values, such as a Range's
    +inf, break the line. The config must set `msg_type`.
    """

    PARAMS = ScalarsParams
    params: ScalarsParams  # type hint for self.params

    def __init__(self, node: Node, config: ModuleConfig, context: BridgeContext) -> None:
        """Subscribe, and check the configured fields against the message type.

        Args:
            node: The ROS node to use for subscriptions & logging.
            config: The module configuration.
            context: Shared context for all modules.
        """
        if not config.msg_type:
            raise ValueError(f"[{config.name}] the scalars module needs `msg_type` set")
        super().__init__(node, config, context)
        msg_type = self._resolve_msg_type()
        for field in self.params.fields:
            _check_field(msg_type, field)
        self._fields = {
            f"{self.entity_path}/{field.replace('.', '/')}": attrgetter(field) for field in self.params.fields
        }
        self._sizes: dict[str, int] = {}
        """Most values logged at once per entity path."""
        if (style := self._series_style()) is not None:
            for path in self._fields:
                rr.log(path, style, static=True)

    def _series_style(self) -> rr.SeriesLines | rr.SeriesPoints | None:
        """The series style from the params, or None to keep Rerun's default lines."""
        p = self.params
        if p.style == "points":
            # Logging any SeriesPoints component is what makes Rerun draw points
            return rr.SeriesPoints(
                names=p.names, colors=p.colours, markers=p.marker or "circle", marker_sizes=p.marker_size
            )
        if all(value is None for value in (p.names, p.colours, p.width, p.interpolation)):
            return None
        interpolation = "".join(word.capitalize() for word in p.interpolation.split("_")) if p.interpolation else None
        return rr.SeriesLines(names=p.names, colors=p.colours, widths=p.width, interpolation_mode=interpolation)

    def _extract_time(self, msg: Any) -> Time | None:
        header = getattr(msg, "header", None)
        return Time.from_msg(header.stamp) if header is not None else None

    def handle(self, msg: Any) -> None:
        for path, get in self._fields.items():
            values = np.atleast_1d(np.asarray(get(msg), dtype=np.float64))
            # Rerun expects as many values on an entity every time: a shorter array is padded with NaN, which leaves a
            # gap in the series it lacks
            size = self._sizes[path] = max(self._sizes.get(path, 0), values.size)
            rr.log(path, rr.Scalars(np.pad(values, (0, size - values.size), constant_values=np.nan)))


def _check_field(msg_type: type, field: str) -> None:
    """Raise a ValueError unless a dotted field path names a number, or an array of numbers, in a message type.

    Args:
        msg_type: ROS message class.
        field: Dotted path, e.g. `twist.twist.linear.x`.
    """
    cls = msg_type
    names = field.split(".")
    for i, name in enumerate(names):
        if not hasattr(cls, "get_fields_and_field_types"):
            raise ValueError(f"field '{field}': '{'.'.join(names[:i])}' is not a message")
        field_type = cls.get_fields_and_field_types().get(name)
        if field_type is None:
            raise ValueError(f"field '{field}': {cls.__name__} has no field '{name}'")
        # A default message holds every nested message, so the next level's class comes from it
        cls = type(getattr(cls(), name))
    array = _ARRAY.fullmatch(field_type)
    if (array.group(1) or array.group(2) if array else field_type) not in _NUMERIC:
        raise ValueError(f"field '{field}' is {field_type}, not a number or an array of numbers")
