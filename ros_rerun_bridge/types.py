from __future__ import annotations

from typing import List

from pydantic import BaseModel, ConfigDict, Field


class StrictParamsModel(BaseModel):
    """Base parameter class with extra fields forbidden."""

    model_config = ConfigDict(extra="forbid")


class TFPath(StrictParamsModel):
    """A TF path specification for rendering poses in Rerun."""

    path: str
    """Rerun entity path where the transform will be logged."""
    child_frame: str
    """TF child frame."""
    parent_frame: str
    """TF parent frame."""


class BaseModelWithTFPaths(StrictParamsModel):
    """Base parameter with a field oflist of TF paths."""

    tf_paths: List[TFPath] = Field(default_factory=list)
