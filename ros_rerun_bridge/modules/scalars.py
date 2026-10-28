from __future__ import annotations

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
from ..utils.fields import check_field, is_array


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
    variance: str | None = None
    """Dotted path to the field's variance, a single number, e.g. `variance`. Drawn as a band one standard deviation
    either side. Needs exactly one field."""
    unit: str | None = None
    """Unit of the values, e.g. `m`, shown in the plot."""

    @property
    def measurements(self) -> bool:
        """Whether values are plotted as Rerun measurements, which carry a variance and a unit."""
        return self.variance is not None or self.unit is not None

    @model_validator(mode="after")
    def _check_style(self) -> ScalarsParams:
        """Reject options that only apply to the other style, or that measurements cannot use."""
        if self.measurements and self.style == "points":
            raise ValueError("variance and unit are drawn as lines, and cannot be used with style 'points'")
        if self.variance is not None and len(self.fields) != 1:
            raise ValueError("variance needs exactly one field")
        other = ("marker", "marker_size") if self.style == "lines" else ("width", "interpolation")
        if given := [name for name in other if getattr(self, name) is not None]:
            raise ValueError(f"{', '.join(given)} cannot be used with style '{self.style}'")
        return self


@REGISTRY.register("scalars")
class ScalarsModule(TopicToComponentModule):
    """Module for plotting numeric fields of any message type as time series.

    Each field is logged on `<entity_path>/<field>`, with the field's dots as slashes. An array is plotted as one series
    per element, with a gap in a series while an array is too short to have it. Non-finite values, such as a Range's
    +inf, break the line. With a `variance` or a `unit`, values are logged as Rerun measurements instead of scalars. The
    config must set `msg_type`.
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
            check_field(msg_type, field)
        if self.params.variance is not None:
            # One variance, applied to every element: arrays of variances could mismatch the values on every message
            if is_array(variance_type := check_field(msg_type, self.params.variance)):
                raise ValueError(f"variance '{self.params.variance}' is {variance_type}, not a single number")
        self._variance = attrgetter(self.params.variance) if self.params.variance is not None else None
        self._fields = {
            f"{self.entity_path}/{field.replace('.', '/')}": attrgetter(field) for field in self.params.fields
        }
        self._sizes: dict[str, int] = {}
        """Most values logged at once per entity path."""
        if (style := self._series_style()) is not None:
            for path in self._fields:
                rr.log(path, style, static=True)

    def _series_style(self) -> rr.SeriesLines | rr.SeriesPoints | rr.Measurements | None:
        """The series style from the params, or None to keep Rerun's default lines."""
        p = self.params
        if p.style == "points":
            # Logging any SeriesPoints component is what makes Rerun draw points
            return rr.SeriesPoints(
                names=p.names, colors=p.colours, markers=p.marker or "circle", marker_sizes=p.marker_size
            )
        interpolation = "".join(word.capitalize() for word in p.interpolation.split("_")) if p.interpolation else None
        if p.measurements:
            # Measurements carry their own style, and the unit with it
            units = [p.unit] if p.unit is not None else None
            return rr.Measurements.from_fields(
                names=p.names, colors=p.colours, widths=p.width, interpolation_mode=interpolation, units=units
            )
        if all(value is None for value in (p.names, p.colours, p.width, p.interpolation)):
            return None
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
            padded = np.pad(values, (0, size - values.size), constant_values=np.nan)
            if not self.params.measurements:
                rr.log(path, rr.Scalars(padded))
            elif self._variance is None:
                rr.log(path, rr.Measurements.from_fields(values=padded))
            else:
                # One variance applies to every element of an array
                variances = np.broadcast_to(np.asarray(self._variance(msg), dtype=np.float64), values.shape)
                variances = np.pad(variances, (0, size - values.size), constant_values=np.nan)
                rr.log(path, rr.Measurements.from_fields(values=padded, variances=variances))
