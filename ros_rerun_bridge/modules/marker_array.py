from __future__ import annotations

import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..utils.utils import set_rr_time_from_ros, stamp_or_now


def _rgba(colour) -> np.ndarray:
    """Convert a std_msgs/ColorRGBA (0-1 floats) to RGBA bytes."""
    return np.round(np.clip([colour.r, colour.g, colour.b, colour.a], 0.0, 1.0) * 255).astype(np.uint8)


def archetype_from_marker(marker: Marker):
    """Convert a marker to a Rerun archetype, in the marker's own frame (its pose applied separately).

    Args:
        marker: An ADD marker.

    Returns:
        The archetype, or None for marker types not supported (MESH_RESOURCE).
    """
    s = marker.scale
    points = np.array([[p.x, p.y, p.z] for p in marker.points], dtype=np.float32).reshape(-1, 3)
    # Per-point colours apply when there is one per point, as in rviz
    colours = (
        np.array([_rgba(c) for c in marker.colors]) if len(marker.colors) == len(points) > 0 else _rgba(marker.color)
    )
    solid = rr.components.FillMode.Solid

    if marker.type == Marker.ARROW:
        if len(points) == 2:
            return rr.Arrows3D(
                origins=points[:1], vectors=points[1:] - points[:1], radii=s.x / 2, colors=_rgba(marker.color)
            )
        return rr.Arrows3D(origins=[[0, 0, 0]], vectors=[[s.x, 0, 0]], radii=s.y / 2, colors=_rgba(marker.color))
    if marker.type == Marker.CUBE:
        return rr.Boxes3D(half_sizes=[[s.x / 2, s.y / 2, s.z / 2]], colors=_rgba(marker.color), fill_mode=solid)
    if marker.type == Marker.SPHERE:
        return rr.Ellipsoids3D(half_sizes=[[s.x / 2, s.y / 2, s.z / 2]], colors=_rgba(marker.color), fill_mode=solid)
    if marker.type == Marker.CYLINDER:
        return rr.Cylinders3D(lengths=[s.z], radii=[s.x / 2], colors=_rgba(marker.color), fill_mode=solid)
    if marker.type == Marker.LINE_STRIP:
        return rr.LineStrips3D([points], radii=s.x / 2, colors=_rgba(marker.color))
    if marker.type == Marker.LINE_LIST:
        # One strip per segment, coloured by its first point
        segment_colours = colours[0 : 2 * (len(points) // 2) : 2] if colours.ndim == 2 else colours
        return rr.LineStrips3D(
            points[: 2 * (len(points) // 2)].reshape(-1, 2, 3), radii=s.x / 2, colors=segment_colours
        )
    # A shape batch has as many instances as half sizes: one each, or every centre after the first is dropped
    list_half_sizes = np.tile([s.x / 2, s.y / 2, s.z / 2], (len(points), 1))
    if marker.type == Marker.CUBE_LIST:
        return rr.Boxes3D(centers=points, half_sizes=list_half_sizes, colors=colours, fill_mode=solid)
    if marker.type == Marker.SPHERE_LIST:
        return rr.Ellipsoids3D(centers=points, half_sizes=list_half_sizes, colors=colours, fill_mode=solid)
    if marker.type == Marker.POINTS:
        return rr.Points3D(points, radii=s.x / 2, colors=colours)
    if marker.type == Marker.TEXT_VIEW_FACING:
        # Rerun has no 3D text: a label on a point, which faces the camera like rviz's text
        return rr.Points3D([[0, 0, 0]], radii=0.0, colors=_rgba(marker.color), labels=[marker.text], show_labels=True)
    if marker.type == Marker.TRIANGLE_LIST:
        n = 3 * (len(points) // 3)
        vertices = points[:n] * [s.x, s.y, s.z]
        # clear_unset: a marker switching between per-vertex colours and one colour must not keep the other
        if colours.ndim == 2:
            return rr.Mesh3D.from_fields(clear_unset=True, vertex_positions=vertices, vertex_colors=colours[:n])
        return rr.Mesh3D.from_fields(clear_unset=True, vertex_positions=vertices, albedo_factor=colours)
    return None


def _ns_part(ns: str) -> str:
    """Return the entity path part for a marker namespace: distinct for every namespace, including "" and "_"."""
    return rr.escape_entity_path_part(f"_{ns}" if not ns or ns.startswith("_") else ns)


@REGISTRY.register("marker_array")
class MarkerArrayModule(TopicToComponentModule):
    """Module for logging visualization_msgs/MarkerArray (or Marker, via `msg_type`) to Rerun, as rviz shows them.

    Each marker is logged on `<entity_path>/<ns>/<id>`, posed in its header frame. Supports every marker type except
    MESH_RESOURCE, the ADD, DELETE and DELETEALL actions, and marker lifetimes. Markers always move with their frame,
    as if `frame_locked` were set.
    """

    def __init__(self, *a, **kw) -> None:
        """Initialise the module with its marker bookkeeping and the lifetime timer."""
        super().__init__(*a, **kw)
        self._types: dict[tuple[str, int], int] = {}
        """Type last logged per marker (ns, id), to clear the old shape when a marker changes type."""
        self._expiries: dict[tuple[str, int], tuple[int, int]] = {}
        """Per marker with a lifetime: when to clear it (node time, ns), and the time to log the clear at (ns)."""
        self.node.create_timer(0.1, self._expire, callback_group=self.callback_group)

    @classmethod
    def ros_msg_type(cls):
        return MarkerArray

    def handle(self, msg: MarkerArray | Marker) -> None:
        for marker in msg.markers if isinstance(msg, MarkerArray) else [msg]:
            stamp = stamp_or_now(Time.from_msg(marker.header.stamp), self.node)
            set_rr_time_from_ros(stamp)
            key = (marker.ns, marker.id)
            if marker.action == Marker.ADD:
                self._add(key, marker, stamp)
            elif marker.action == Marker.DELETE:
                self._clear([key])
            elif marker.action == Marker.DELETEALL:
                # As rviz: only the message's namespace, or every marker if it has none
                ns_path = f"{self.entity_path}/{_ns_part(marker.ns)}" if marker.ns else self.entity_path
                rr.log(ns_path, rr.Clear(recursive=True))
                for k in [k for k in {*self._types, *self._expiries} if not marker.ns or k[0] == marker.ns]:
                    self._types.pop(k, None)
                    self._expiries.pop(k, None)

    def _path(self, key: tuple[str, int]) -> str:
        return f"{self.entity_path}/{_ns_part(key[0])}/{key[1]}"

    def _add(self, key: tuple[str, int], marker: Marker, stamp: Time) -> None:
        archetype = archetype_from_marker(marker)
        if archetype is None:
            self.node.get_logger().warn(
                f"[{self.name}] marker type {marker.type} is not supported", throttle_duration_sec=5.0
            )
            return
        path = self._path(key)
        if self._types.get(key, marker.type) != marker.type:
            rr.log(path, rr.Clear(recursive=False))
        self._types[key] = marker.type
        # The marker's pose places its own frame in the header frame
        p, q = marker.pose.position, marker.pose.orientation
        # rviz treats an all-zero orientation, common in hand-written publishers, as identity
        quaternion = [q.x, q.y, q.z, q.w] if (q.x, q.y, q.z, q.w) != (0.0, 0.0, 0.0, 0.0) else [0.0, 0.0, 0.0, 1.0]
        marker_frame = f"marker:{path}"
        rr.log(
            path,
            rr.Transform3D(
                translation=[p.x, p.y, p.z],
                quaternion=quaternion,
                parent_frame=marker.header.frame_id or None,
                child_frame=marker_frame,
            ),
            rr.CoordinateFrame(frame=marker_frame),
            archetype,
        )
        lifetime = Duration.from_msg(marker.lifetime).nanoseconds
        if lifetime > 0:
            # As rviz, the lifetime runs from when the marker is received; the clear is logged relative to its stamp
            self._expiries[key] = (self.node.get_clock().now().nanoseconds + lifetime, stamp.nanoseconds + lifetime)
        else:
            self._expiries.pop(key, None)

    def _clear(self, keys: list[tuple[str, int]]) -> None:
        for key in keys:
            rr.log(self._path(key), rr.Clear(recursive=True))
            self._types.pop(key, None)
            self._expiries.pop(key, None)

    def _expire(self) -> None:
        """Clear markers whose lifetime has run out, logged at their stamp plus lifetime."""
        now = self.node.get_clock().now().nanoseconds
        for key, (deadline, log_time) in [(k, e) for k, e in self._expiries.items() if e[0] <= now]:
            set_rr_time_from_ros(Time(nanoseconds=log_time))
            self._clear([key])
