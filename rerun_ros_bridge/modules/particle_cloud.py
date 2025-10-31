from __future__ import annotations

from typing import Any, List, Tuple

import numpy as np
from rclpy.time import Time
from scipy.spatial.transform import Rotation

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY


@REGISTRY.register("particle_cloud")
class ParticleCloudModule(TopicToComponentModule):
    """
    Visualize nav2_msgs/ParticleCloud as:
      - mode: "points"  -> rr.Points3D colored by weight
      - mode: "arrows"  -> rr.LineStrips3D short arrows oriented by pose and scaled by weight
    Extra config (all optional):
      mode: "points" | "arrows"  (default: "points")
      colour: [r,g,b,a]     (uint8, default: [255,255,255,255])
      colour_by_weight: true|false (default: false)  # if true, modulate alpha by normalized weight
      w_min: float (default: auto from current msg)  # normalization floor
      w_max: float (default: auto from current msg)  # normalization ceil
      arrow_len: float (default: 0.15)               # base arrow length (meters) before weight scaling
      arrow_radius: float (default: 0.002)           # base radius for LineStrips3D
      arrow_axis: "x"|"y"|"z" (default: "x")        # which body axis defines heading
      tf_paths: [ {path, child_frame, parent_frame}, ... ]  # same pattern as other modules
    """

    @classmethod
    def ros_msg_type(cls):
        from nav2_msgs.msg import ParticleCloud

        return ParticleCloud

    def _extract_time(self, msg) -> Time | None:  # type: ignore[override]
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: Any) -> None:
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
        w_min = float(self.extra.get("w_min", np.min(w)))
        w_max = float(self.extra.get("w_max", np.max(w)))
        w_norm = np.ones_like(w) if w_max <= w_min else (w - w_min) / (w_max - w_min)
        w_norm = np.clip(w_norm, 0.0, 1.0)

        mode = (self.extra.get("mode") or "points").lower()
        colour_by_weight = bool(self.extra.get("colour_by_weight", False))

        # base color
        base = self.extra.get("colour", [255, 255, 255, 255])
        r, g, b, a = [int(x) for x in base]
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

        if mode == "arrows":
            axis = (self.extra.get("arrow_axis") or "x").lower()
            base_len = float(self.extra.get("arrow_len", 0.15))
            base_rad = float(self.extra.get("arrow_radius", 0.002))

            q_xyzw = np.asarray(quats_xyzw, dtype=np.float32)
            rot = Rotation.from_quat(q_xyzw)  # SciPy expects xyzw

            axis = (self.extra.get("arrow_axis") or "x").lower()
            basis = {"x": np.array([1, 0, 0]), "y": np.array([0, 1, 0]), "z": np.array([0, 0, 1])}[axis]

            dirs = rot.apply(np.tile(basis, (len(particles), 1)))

            base_len = float(self.extra.get("arrow_len", 0.15))
            base_rad = float(self.extra.get("arrow_radius", 0.002))
            lengths = (base_len * (0.2 + 0.8 * w_norm)).reshape(-1, 1)  # keep a small floor
            vectors = dirs * lengths  # (N,3)

            rr.log(self.entity_path, rr.Arrows3D(origins=pos, vectors=vectors, radii=base_rad, colors=colours))
        else:
            # POINTS MODE
            rr.log(self.entity_path, rr.Points3D(pos, colours=colours))

        # Optional TF attachments (same pattern as other modules)
        stamp = self._extract_time(msg)
        for tfp in self.extra.get("tf_paths", []):
            self.context.tf.log_tf_path(
                path=tfp["path"],
                child_frame=tfp["child_frame"],
                parent_frame=tfp["parent_frame"],
                time=stamp,
            )
