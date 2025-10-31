from __future__ import annotations

from typing import List, Literal, Tuple

import numpy as np
from rclpy.time import Time
from scipy.spatial.transform import Rotation

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import BaseModelWithTFPaths

try:
    from nav2_msgs.msg import ParticleCloud

    NAV2_MSGS_AVAILABLE = True
except ImportError:
    NAV2_MSGS_AVAILABLE = False


class ParticleCloudParams(BaseModelWithTFPaths):
    """Parameters for the ParticleCloudModule."""

    mode: Literal["points", "arrows"] = "points"
    """Visualization mode. 'points' for Points3D, 'arrows' for LineStrips3D."""
    radius: float = 0.0015
    """Base radius for Points3D or LineStrips3D."""
    colour: tuple[int, int, int, int] = (255, 255, 255, 255)
    """Base RGBA colour as 4-tuple of uint8."""
    colour_by_weight: bool = True
    """Whether to modulate colour alpha by weight."""
    w_min: float | None = None
    """Minimum weight for normalization (default: auto from current msg)."""
    w_max: float | None = None
    """Maximum weight for normalization (default: auto from current msg)."""
    arrow_len: float = 0.05
    """Base arrow length in meters (before weight scaling)."""
    arrow_axis: Literal["x", "y", "z"] = "x"
    """Which body axis defines heading for arrows."""


@REGISTRY.register("particle_cloud")
class ParticleCloudModule(TopicToComponentModule):
    """
    Visualize nav2_msgs/ParticleCloud as:
        - mode: "points"  -> rr.Points3D coloured by weight
        - mode: "arrows"  -> rr.LineStrips3D short arrows oriented by pose and scaled by weight
    Module-specific extra configuration parameters:
        - mode: "points" | "arrows"  (default: "points")
                - points: render as points
                - arrows: render as arrows oriented by pose
        - radius: float (default: 0.0015)
                - base radius for points or arrows
        - colour: tuple[int, int, int, int] (default: (255, 255, 255, 255))
                - base RGBA colour as 4-tuple of uint8
        - colour_by_weight: bool (default: True)
                - whether to modulate colour alpha by weight
        - w_min: float | None (default: None)
                - minimum weight for normalization (auto from current msg if None)
        - w_max: float | None (default: None)
                - maximum weight for normalization (auto from current msg if None)
        - arrow_len: float (default: 0.05)
                - base arrow length in meters (before weight scaling)
        - arrow_axis: "x" | "y" | "z" (default: "x")
                - which body axis defines heading for arrows
    """

    PARAMS = ParticleCloudParams
    params: ParticleCloudParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):  # noqa: D102
        if not NAV2_MSGS_AVAILABLE:
            raise RuntimeError("nav2_msgs package is not installed.")

        return ParticleCloud

    def _extract_time(self, msg: ParticleCloud) -> Time | None:  # type: ignore[override]
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: ParticleCloud) -> None:  # noqa: D102
        particles = msg.particles or []
        if not particles:
            return

        # collect poses & weights
        positions: List[Tuple[float, float, float]] = []
        quats_xyzw: List[Tuple[float, float, float, float]] = []
        weights: List[float] = []
        for p in particles:
            t = p.pose.position
            q = p.pose.orientation
            positions.append((t.x, t.y, t.z))
            quats_xyzw.append((q.x, q.y, q.z, q.w))
            weights.append(float(p.weight))

        pos = np.asarray(positions, dtype=np.float32)  # (N,3)
        w = np.asarray(weights, dtype=np.float32)  # (N,)

        # weight normalization
        w_min = self.params.w_min if self.params.w_min is not None else float(np.min(w))
        w_max = self.params.w_max if self.params.w_max is not None else float(np.max(w))
        w_norm = np.ones_like(w) if w_max <= w_min else (w - w_min) / (w_max - w_min)
        w_norm = np.clip(w_norm, 0.0, 1.0)

        colour_by_weight = self.params.colour_by_weight

        # base colour
        r, g, b, a = [int(x) for x in self.params.colour]
        if colour_by_weight:
            # modulate alpha by weight (fade weaker particles); avoid fully invisible
            alpha = np.clip((0.15 + 0.85 * w_norm) * 255.0, 1.0, 255.0).astype(np.uint8)
            colours = np.column_stack(
                [
                    np.full_like(alpha, r, dtype=np.uint8),
                    np.full_like(alpha, g, dtype=np.uint8),
                    np.full_like(alpha, b, dtype=np.uint8),
                    alpha,
                ]
            )
        else:
            colours = np.array([[r, g, b, a]], dtype=np.uint8).repeat(pos.shape[0], axis=0)

        if self.params.mode == "arrows":
            q_xyzw = np.asarray(quats_xyzw, dtype=np.float32)
            rot = Rotation.from_quat(q_xyzw)  # SciPy expects xyzw

            basis = {"x": np.array([1, 0, 0]), "y": np.array([0, 1, 0]), "z": np.array([0, 0, 1])}[self.params.arrow_axis]

            dirs = rot.apply(np.tile(basis, (len(particles), 1)))

            lengths = (self.params.arrow_len * (0.2 + 0.8 * w_norm)).reshape(-1, 1)  # keep a small floor
            vectors = dirs * lengths  # (N,3)

            rr.log(self.entity_path, rr.Arrows3D(origins=pos, vectors=vectors, radii=self.params.radius, colors=colours))
        else:
            # POINTS MODE
            rr.log(self.entity_path, rr.Points3D(pos, colours=colours, radii=self.params.radius))

        # Optional TF attachments (same pattern as other modules)
        stamp = self._extract_time(msg)
        for tfp in self.params.tf_paths:
            self.context.tf.log_tf_path(
                tf_path=tfp,
                time=stamp,
            )
