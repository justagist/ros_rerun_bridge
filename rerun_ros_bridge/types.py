from __future__ import annotations

from typing import List

from pydantic import BaseModel, ConfigDict, Field


class StrictParamsModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class TFPath(StrictParamsModel):
    path: str
    child_frame: str
    parent_frame: str


# Common mixin if you want to reuse config options across extras
class BaseModelWithTFPaths(StrictParamsModel):
    tf_paths: List[TFPath] = Field(default_factory=list)
