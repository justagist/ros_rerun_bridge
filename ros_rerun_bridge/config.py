"""
The bridge's config: the modules to run and automatic discovery's settings, validated from YAML.
"""

from __future__ import annotations

import re
from fnmatch import fnmatchcase
from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import Field, PositiveInt, field_validator, model_validator

from .types import StrictParamsModel
from .utils.frame_axes import FrameAxesParams


class QoSConfig(StrictParamsModel):
    """Subscription QoS for a module. The publisher's QoS must be compatible, e.g. `best_effort` for sensor data."""

    depth: PositiveInt = 10
    reliability: Literal["reliable", "best_effort"] = "reliable"
    durability: Literal["volatile", "transient_local"] = "volatile"
    history: Literal["keep_last", "keep_all"] = "keep_last"


class ModuleConfig(StrictParamsModel):
    """Configuration for a single bridge module."""

    name: str
    """Name of the module."""
    module: str
    """registry key or python path to the module class. e.g. 'image' or 'my_pkg.my_mod:MyModule'"""
    topic: str
    """ROS topic to subscribe to."""
    entity_path: str
    """Rerun entity path for logging."""
    msg_type: str | None = None
    """Python path to ROS msg class (overrides module default if provided)."""
    qos: QoSConfig = Field(default_factory=QoSConfig)
    """Quality of Service settings."""
    params: dict = Field(default_factory=dict)
    """Additional parameters for the module."""


class AutoFrameAxesParams(FrameAxesParams):
    """How discovered TF frames are drawn: the `tf` module's frame params, with axes by default."""

    axis_length: float | None = Field(default=0.1, gt=0, allow_inf_nan=False)
    """Draw each frame's axes, this long in metres, or none if null. Names are off by default, as many frames would
    crowd the view, and the viewer can show each frame's name."""


class AutoModule(StrictParamsModel):
    """A module automatic discovery shows the topics of a message type with."""

    module: str
    """A built-in module's name, or the Python path to a module class, e.g. `my_pkg.my_mod:MyModule`."""
    params: dict = Field(default_factory=dict)
    """The module's parameters."""


JointField = Literal["position", "velocity", "effort"]
"""A `JointState` field that can be plotted."""

JointSource = Literal["tf", "joint_states", "both"]
"""Which source moves a joint that arrives both on /tf and as joint states: either, falling back to the other a second
after it stops sending the joint, or both, the latest value winning."""


class AutoJointStates(StrictParamsModel):
    """How discovered `JointState` topics move the robot and are plotted: the `joint_states` module's params."""

    topics: list[str] = Field(default_factory=lambda: ["*"])
    """The `JointState` topics to bridge. The others, e.g. commands, which would draw the robot at its targets, are
    not."""
    prefer: JointSource = "tf"
    """Which source moves a joint that also arrives on /tf, as the `joint_states` module's `prefer`."""
    plot: list[JointField] = Field(default_factory=lambda: ["position"])
    """The fields to plot, on `<topic>/<field>/<joint>`. Position only by default, as each field adds a series per
    joint."""


class AutoConfig(StrictParamsModel):
    """Settings for automatic discovery: which topics it may bridge, as `fnmatch` globs on the topic name (`*` also
    matches `/`), how often it looks, when a topic is stale, and how it draws TF frames."""

    include: list[str] = Field(default_factory=lambda: ["*"])
    """Topics to bridge, if they have a module for their type."""
    exclude: list[str] = Field(default_factory=list)
    """Topics never to bridge, even if included."""
    frames: AutoFrameAxesParams = Field(default_factory=AutoFrameAxesParams)
    """How the frames from /tf and /tf_static are drawn."""
    joint_states: AutoJointStates = Field(default_factory=AutoJointStates)
    """How `JointState` topics move the robot and are plotted."""
    scan_period: float = Field(default=2.0, gt=0, allow_inf_nan=False)
    """Seconds between looks at the ROS graph."""
    stale_scans: PositiveInt = 3
    """Scans in a row a topic may have no publishers before it is stale and dropped. A few, so that a restarting
    publisher keeps its topic."""
    never_stale: list[str] = Field(default_factory=list)
    """Topics never dropped, nor their data cleared, when their publishers go, e.g. a sensor started on demand."""
    modules: dict[str, AutoModule] = Field(default_factory=dict)
    """Modules to show the topics of these message types with, by type, e.g. `my_msgs/msg/Thing`. Each replaces the
    built-in choice for its type."""

    @field_validator("modules", mode="before")
    @classmethod
    def _modules_by_name(cls, value: Any) -> Any:
        """Accept a module's name or path alone, for a module without params, and check the message types' names."""
        if not isinstance(value, dict):
            return value
        for msg_type in value:
            if not re.fullmatch(r"\w+/msg/\w+", msg_type):
                raise ValueError(f"'{msg_type}' is not a message type, e.g. 'nav_msgs/msg/Odometry'")
        return {msg_type: {"module": m} if isinstance(m, str) else m for msg_type, m in value.items()}

    @model_validator(mode="after")
    def _check_joint_states(self) -> AutoConfig:
        """Reject `joint_states` with a module mapped to `JointState`, which replaces the topics' `joint_states`
        modules and would leave every setting in `joint_states` unused."""
        if "joint_states" in self.model_fields_set and "sensor_msgs/msg/JointState" in self.modules:
            raise ValueError(
                "'joint_states' does nothing when 'modules' maps sensor_msgs/msg/JointState: set one or the other"
            )
        return self

    def matches(self, topic: str) -> bool:
        """Whether the topic is included and not excluded."""
        return any(fnmatchcase(topic, p) for p in self.include) and not any(fnmatchcase(topic, p) for p in self.exclude)

    def is_never_stale(self, topic: str) -> bool:
        """Whether the topic stays bridged without publishers."""
        return any(fnmatchcase(topic, p) for p in self.never_stale)


class BridgeConfig(StrictParamsModel):
    """Configuration for the ROS->Rerun bridge."""

    global_frame: str = "map"
    """TF frame that Rerun's default 3D view is rendered in."""
    blueprint: str | None = None
    """Viewer layout to open with: a .rbl file, e.g. saved from the viewer. Relative paths are to the config file."""
    auto: AutoConfig | None = None
    """Automatic discovery of the topics `modules` does not name: `true`, or the topics it may bridge. Off if unset."""
    modules: list[ModuleConfig] = Field(default_factory=list)
    """List of bridge modules."""

    @field_validator("auto", mode="before")
    @classmethod
    def _auto_from_bool(cls, value: Any) -> Any:
        """Accept `auto: true` for discovery with the default filters, and `false` for none."""
        if isinstance(value, bool):
            return AutoConfig() if value else None
        return value


def load_config(path: str | Path) -> BridgeConfig:
    """Load and validate a config, with its blueprint path resolved and checked.

    Args:
        path: Path to the YAML config.

    Returns:
        The config, with `blueprint` an absolute path if set.

    Raises:
        pydantic.ValidationError: If the config is invalid.
        FileNotFoundError: If the config or its blueprint does not exist.
    """
    with open(path) as f:
        cfg = BridgeConfig.model_validate(yaml.safe_load(f))
    if cfg.blueprint:
        blueprint = (Path(path).parent / cfg.blueprint).resolve()
        if not blueprint.is_file():
            raise FileNotFoundError(f"blueprint '{blueprint}' not found")
        cfg.blueprint = str(blueprint)
    return cfg
