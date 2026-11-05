from dataclasses import dataclass, field
from typing import Optional

import rerun as rr

HANDOVER = 1.0
"""Seconds after a joint's preferred source, /tf or joint states, last sent its frame, that the other source moves it
again, e.g. once robot_state_publisher stops."""


@dataclass
class BridgeContext:
    """Shared context for all modules that run at a time."""

    urdf_tree: Optional[rr.urdf.UrdfTree] = None
    """Robot model, filled by the robot_description module once the URDF is received."""
    tf_frame_times: dict[str, float] = field(default_factory=dict)
    """When each child frame last arrived on a non-static `tf` module's topic, by `time.monotonic()`, for
    `joint_states` modules preferring /tf to leave those joints to it."""
    joint_state_frame_times: dict[str, float] = field(default_factory=dict)
    """When each child frame was last moved by a `joint_states` module preferring joint states, by `time.monotonic()`,
    for `tf` modules to leave those frames to it."""
