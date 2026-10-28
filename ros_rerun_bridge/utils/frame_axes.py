"""Drawing frames as axes, for the modules that log transforms: `tf` and `joint_states`."""

from __future__ import annotations

from pydantic import Field, model_validator

import rerun as rr

from ..types import StrictParamsModel


class FrameAxesParams(StrictParamsModel):
    """Parameters for drawing the frames a module logs transforms for, shared by the modules that do."""

    axis_length: float | None = Field(default=None, gt=0, allow_inf_nan=False)
    """Draw each frame's x (red), y (green) and z (blue) axes, this long in metres, as rviz's TF display does."""
    frame_names: bool = False
    """Label each frame's axes with its name. Needs `axis_length`."""

    @model_validator(mode="after")
    def _check_axes(self) -> FrameAxesParams:
        """Frame names are drawn on the axes, so there must be axes."""
        if self.frame_names and self.axis_length is None:
            raise ValueError("frame_names needs axis_length")
        return self


class FrameAxes:
    """Draws each frame's axes, if the params ask for them, on `frames/<frame>`.

    An entity per frame lets the viewer show or hide each frame's axes, and its name, on its own. The path is the same
    for every module, so a frame that several modules see, e.g. from /tf_static and /joint_states, has one entity.
    """

    def __init__(self, params: FrameAxesParams) -> None:
        """Set up drawing frames.

        Args:
            params: The module's frame axes params.
        """
        self._params = params
        self._drawn: set[str] = set()

    def add(self, *frames: str) -> None:
        """Draw frames not drawn yet. Their axes then follow the frames as their transforms change.

        Args:
            frames: Frame names, e.g. a transform's parent and child frames.
        """
        if self._params.axis_length is None:
            return
        for frame in frames:
            if frame and frame not in self._drawn:
                rr.log(
                    f"frames/{rr.escape_entity_path_part(frame)}",
                    rr.CoordinateFrame(frame=frame),
                    rr.TransformAxes3D(axis_length=self._params.axis_length, show_frame=self._params.frame_names),
                    static=True,
                )
                self._drawn.add(frame)
