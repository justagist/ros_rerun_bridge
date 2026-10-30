from __future__ import annotations

import json
import re
from functools import lru_cache

import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import StrictParamsModel
from ..utils.meshes import collada_unit, mesh_path
from ..utils.utils import set_rr_time_from_ros, stamp_or_now

# The mesh formats Rerun's viewer loads. Its SDK guesses the media type of each but Collada.
_MESH_MEDIA_TYPES = {
    ".dae": "model/vnd.collada+xml",
    ".glb": rr.components.MediaType.GLB,
    ".gltf": rr.components.MediaType.GLTF,
    ".obj": rr.components.MediaType.OBJ,
    ".ply": rr.components.MediaType.PLY,
    ".stl": rr.components.MediaType.STL,
}
_COLLADA_VERSION = re.compile(rb"""<COLLADA\b[^>]*\bversion\s*=\s*["']([^"']+)""")


def _rgba(colour) -> np.ndarray:
    """Convert a std_msgs/ColorRGBA (0-1 floats) to RGBA bytes."""
    return np.round(np.clip([colour.r, colour.g, colour.b, colour.a], 0.0, 1.0) * 255).astype(np.uint8)


def archetype_from_marker(marker: Marker):
    """Convert a marker to a Rerun archetype, in the marker's own frame (its pose applied separately).

    Args:
        marker: An ADD marker.

    Returns:
        The archetype, or None for marker types not supported here: MESH_RESOURCE, which `mesh_from_marker` converts,
        and ARROW_STRIP, which rviz does not draw either.
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
        if colours.ndim == 2:
            # A Rerun strip has one colour: one strip per segment, coloured by its first point
            segments = np.stack([points[:-1], points[1:]], axis=1)
            return rr.LineStrips3D(segments, radii=s.x / 2, colors=colours[:-1])
        return rr.LineStrips3D([points], radii=s.x / 2, colors=colours)
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
        # clear_unset: a marker switching between per-vertex colours and one colour must not keep the other. It also
        # logs unset indices as an empty list, which Rerun draws as no triangles, so they are always given.
        indices = np.arange(n, dtype=np.uint32).reshape(-1, 3)
        if colours.ndim == 2:
            return rr.Mesh3D.from_fields(
                clear_unset=True, vertex_positions=vertices, triangle_indices=indices, vertex_colors=colours[:n]
            )
        return rr.Mesh3D.from_fields(
            clear_unset=True, vertex_positions=vertices, triangle_indices=indices, albedo_factor=colours
        )
    return None


def mesh_from_marker(marker: Marker) -> tuple[rr.Asset3D, list[float]]:
    """Convert a MESH_RESOURCE marker to a Rerun asset, in the marker's own frame, and the scale to draw it at.

    As in rviz, the marker's colour replaces the mesh's own colours unless `mesh_use_embedded_materials` is set. Rerun
    multiplies them instead, which is the same for meshes without colours of their own, e.g. most STL files. Embedded
    materials are not tinted by a non-zero colour, as rviz does.

    Args:
        marker: An ADD MESH_RESOURCE marker.

    Returns:
        The asset, and the marker's scale, including the unit a Collada mesh declares.

    Raises:
        ValueError: If the mesh cannot be loaded.
    """
    contents, media_type, unit = _load_mesh(marker.mesh_resource)
    colour = None if marker.mesh_use_embedded_materials else _rgba(marker.color)
    s = marker.scale
    scale = [s.x * unit, s.y * unit, s.z * unit]
    # clear_unset: a marker switching to embedded materials must not keep its old colour
    return rr.Asset3D.from_fields(clear_unset=True, blob=contents, media_type=media_type, albedo_factor=colour), scale


@lru_cache(maxsize=64)
def _load_mesh(resource: str) -> tuple[bytes, str, float]:
    """Read a mesh once, however many markers or messages use it.

    Args:
        resource: The marker's `mesh_resource`: `package://`, `file://` or absolute.

    Returns:
        The mesh file's contents, its media type, and the metres per unit it declares.

    Raises:
        ValueError: If the mesh cannot be found, is in a format or Collada version Rerun does not load, or is a glTF
            file that refers to other files.
        OSError: If the mesh cannot be read.
    """
    if "://" in resource and not resource.startswith(("package://", "file://")):
        raise ValueError(f"mesh '{resource}' is not a package:// or file:// path")
    path = mesh_path(resource)
    if path is None or not path.is_file():
        raise ValueError(f"mesh '{resource}' not found")
    suffix = path.suffix.lower()
    if suffix not in _MESH_MEDIA_TYPES:
        raise ValueError(f"mesh '{resource}' is not in a format Rerun loads (Collada, glTF, OBJ, PLY or STL)")
    contents = path.read_bytes()
    # Rerun's viewer only reports these itself, where it may not be seen, e.g. on another machine
    if suffix == ".dae":
        version = _COLLADA_VERSION.search(contents[:16384])
        if version and version.group(1) != b"1.4.1":
            raise ValueError(f"mesh '{resource}' is Collada {version.group(1).decode()}, and Rerun only loads 1.4.1")
        # Rerun's Collada importer ignores the unit, which rviz applies
        return contents, _MESH_MEDIA_TYPES[suffix], collada_unit(resource)
    if suffix == ".gltf":
        # Rerun is sent the file alone, so it cannot load the buffers and images it refers to
        gltf = json.loads(contents)
        uris = [item.get("uri") for item in gltf.get("buffers", []) + gltf.get("images", [])]
        if any(uri and not uri.startswith("data:") for uri in uris):
            raise ValueError(f"mesh '{resource}' refers to other files, which Rerun cannot load: use a .glb file")
    return contents, _MESH_MEDIA_TYPES[suffix], 1.0


def _ns_part(ns: str) -> str:
    """Return the entity path part for a marker namespace: distinct for every namespace, including "" and "_"."""
    return rr.escape_entity_path_part(f"_{ns}" if not ns or ns.startswith("_") else ns)


@REGISTRY.register("marker_array")
class MarkerArrayModule(TopicToComponentModule):
    """Module for logging visualization_msgs/MarkerArray (or Marker, via `msg_type`) to Rerun, as rviz shows them.

    Each marker is logged on `<entity_path>/<ns>/<id>`, posed in its header frame. Supports every marker type except
    ARROW_STRIP, which rviz does not draw either, the ADD, DELETE and DELETEALL actions, and marker lifetimes. Markers
    always move with their frame, as if `frame_locked` were set.
    """

    # No params, so any given are an error
    PARAMS = StrictParamsModel

    def __init__(self, *a, **kw) -> None:
        """Initialise the module with its marker bookkeeping and the lifetime timer."""
        super().__init__(*a, **kw)
        self._shapes: dict[tuple[str, int], tuple[int, type]] = {}
        """Type and archetype class last logged per marker (ns, id), to clear its shape on a type change or deletion."""
        self._expiries: dict[tuple[str, int], tuple[int, int]] = {}
        """Per marker with a lifetime: when to clear it (node time, ns), and the time to log the clear at (ns)."""
        self._meshes: dict[tuple[str, int], tuple[tuple, int, rr.Asset3D]] = {}
        """Per MESH_RESOURCE marker (ns, id), its latest stamped mesh, the stamp (ns) and asset, so a mesh is only
        logged again when it changes."""
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
                self._clear([k for k in self._shapes if not marker.ns or k[0] == marker.ns])

    def _path(self, key: tuple[str, int]) -> str:
        return f"{self.entity_path}/{_ns_part(key[0])}/{key[1]}"

    def _add(self, key: tuple[str, int], marker: Marker, stamp: Time) -> None:
        scale = None
        if marker.type == Marker.MESH_RESOURCE:
            try:
                archetype, scale = mesh_from_marker(marker)
                error = None
            except (ValueError, OSError) as e:
                archetype, error = None, str(e)
        else:
            archetype = archetype_from_marker(marker)
            error = f"marker type {marker.type} is not supported"
        if archetype is None:
            self.node.get_logger().warn(f"[{self.name}] {error}", throttle_duration_sec=5.0)
            # A marker that cannot be drawn must not keep showing its old shape
            self._clear([key])
            return
        path = self._path(key)
        previous = self._shapes.get(key)
        if previous is not None and previous[0] != marker.type:
            self._clear([key])
        self._shapes[key] = (marker.type, type(archetype))
        archetypes, relog = (archetype,), None
        if marker.type == Marker.MESH_RESOURCE:
            mesh = (marker.mesh_resource, marker.mesh_use_embedded_materials, tuple(_rgba(marker.color)))
            latest = self._meshes.get(key)
            if latest is None or stamp.nanoseconds >= latest[1]:
                # A mesh can be large, and markers are often republished unchanged: log it only when it changes
                if latest is not None and latest[0] == mesh:
                    archetypes = ()
                self._meshes[key] = (mesh, stamp.nanoseconds, archetype)
            else:
                # A late message, e.g. from a replayed bag, is logged at its stamp, where its mesh would also stand for
                # the unchanged meshes not logged after it: the latest one is logged again at its stamp
                relog = latest
        # The marker's pose places its own frame in the header frame
        p, q = marker.pose.position, marker.pose.orientation
        # rviz treats an all-zero orientation, common in hand-written publishers, as identity
        quaternion = [q.x, q.y, q.z, q.w] if (q.x, q.y, q.z, q.w) != (0.0, 0.0, 0.0, 0.0) else [0.0, 0.0, 0.0, 1.0]
        marker_frame = f"marker:{path}"
        rr.log(
            path,
            # clear_unset: a mesh's scale must not stay on a marker that changes type
            rr.Transform3D.from_fields(
                clear_unset=True,
                translation=[p.x, p.y, p.z],
                quaternion=quaternion,
                scale=scale,
                parent_frame=marker.header.frame_id or None,
                child_frame=marker_frame,
            ),
            rr.CoordinateFrame(frame=marker_frame),
            *archetypes,
        )
        if relog is not None:
            set_rr_time_from_ros(Time(nanoseconds=relog[1]))
            rr.log(path, relog[2])
        lifetime = Duration.from_msg(marker.lifetime).nanoseconds
        if lifetime > 0:
            # As rviz, the lifetime runs from when the marker is received; the clear is logged relative to its stamp
            self._expiries[key] = (self.node.get_clock().now().nanoseconds + lifetime, stamp.nanoseconds + lifetime)
        else:
            self._expiries.pop(key, None)

    def _clear(self, keys: list[tuple[str, int]]) -> None:
        """Clear markers' shapes, keeping their transforms.

        An `rr.Clear` would also clear the marker's frame, and Rerun keeps a cleared frame over a transform logged at
        the same time: a marker re-added, or changing type, with the same stamp would never be drawn.
        """
        for key in keys:
            shape = self._shapes.pop(key, None)
            if shape is not None:
                rr.log(self._path(key), shape[1].cleared())
            self._expiries.pop(key, None)
            self._meshes.pop(key, None)

    def _expire(self) -> None:
        """Clear markers whose lifetime has run out, logged at their stamp plus lifetime."""
        now = self.node.get_clock().now().nanoseconds
        for key, (deadline, log_time) in [(k, e) for k, e in self._expiries.items() if e[0] <= now]:
            set_rr_time_from_ros(Time(nanoseconds=log_time))
            self._clear([key])
