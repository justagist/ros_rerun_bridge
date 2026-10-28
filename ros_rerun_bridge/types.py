from __future__ import annotations

from typing import Annotated

from pydantic import BaseModel, ConfigDict, Field

_Channel = Annotated[int, Field(ge=0, le=255)]
Rgba = tuple[_Channel, _Channel, _Channel, _Channel]
"""An RGBA colour as four 0-255 channels. Rerun would wrap an out-of-range channel round instead of rejecting it."""


class StrictParamsModel(BaseModel):
    """Base parameter class with extra fields forbidden."""

    model_config = ConfigDict(extra="forbid")
