from __future__ import annotations

from pydantic import BaseModel, ConfigDict


class StrictParamsModel(BaseModel):
    """Base parameter class with extra fields forbidden."""

    model_config = ConfigDict(extra="forbid")
