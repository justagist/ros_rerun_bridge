from pathlib import Path

import numpy as np
import pytest
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import rerun as rr

from ros_rerun_bridge.modules.marker_array import archetype_from_marker

DATA_DIR = Path(__file__).parent / "data"

POINTS = [
    Point(x=0.0, y=0.0, z=0.0),
    Point(x=1.0, y=0.0, z=0.0),
    Point(x=1.0, y=1.0, z=0.0),
    Point(x=0.0, y=1.0, z=0.0),
]
PER_POINT = [ColorRGBA(r=1.0, a=1.0), ColorRGBA(g=1.0, a=1.0), ColorRGBA(b=1.0, a=1.0), ColorRGBA(a=1.0)]


def marker(marker_type: int = Marker.CUBE, ns: str = "ns", marker_id: int = 0, **fields) -> Marker:
    m = Marker(type=marker_type, ns=ns, id=marker_id, **fields)
    m.header.frame_id = "table"
    m.scale.x, m.scale.y, m.scale.z = 0.2, 0.4, 0.6
    m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
    m.pose.orientation.w = 1.0
    return m


def component(archetype, name: str):
    return getattr(archetype, name).as_arrow_array().to_pylist()


@pytest.mark.parametrize(
    ("marker_type", "archetype", "instances", "count"),
    [
        (Marker.ARROW, rr.Arrows3D, "vectors", 1),
        (Marker.CUBE, rr.Boxes3D, "half_sizes", 1),
        (Marker.SPHERE, rr.Ellipsoids3D, "half_sizes", 1),
        (Marker.CYLINDER, rr.Cylinders3D, "lengths", 1),
        (Marker.LINE_STRIP, rr.LineStrips3D, "strips", 1),
        (Marker.LINE_LIST, rr.LineStrips3D, "strips", 2),
        (Marker.CUBE_LIST, rr.Boxes3D, "half_sizes", 4),
        (Marker.SPHERE_LIST, rr.Ellipsoids3D, "half_sizes", 4),
        (Marker.POINTS, rr.Points3D, "positions", 4),
        (Marker.TEXT_VIEW_FACING, rr.Points3D, "positions", 1),
        (Marker.TRIANGLE_LIST, rr.Mesh3D, "vertex_positions", 3),
    ],
)
def test_every_marker_type_rviz_draws_but_mesh_resource_converts_to_its_instances(
    marker_type, archetype, instances, count
):
    # Four points: list markers draw one instance per point (or per segment, or per whole triangle)
    converted = archetype_from_marker(marker(marker_type, points=POINTS, text="t"))
    assert isinstance(converted, archetype)
    assert len(component(converted, instances)) == count


def test_arrow_strips_are_not_converted_as_rviz_does_not_draw_them():
    assert archetype_from_marker(marker(Marker.ARROW_STRIP, points=POINTS)) is None


def test_line_strips_with_per_point_colours_draw_a_segment_per_colour():
    strip = archetype_from_marker(marker(Marker.LINE_STRIP, points=POINTS[:3], colors=PER_POINT[:3]))
    assert component(strip, "strips") == [[[0, 0, 0], [1, 0, 0]], [[1, 0, 0], [1, 1, 0]]]
    # Each segment takes its first point's colour
    assert component(strip, "colors") == [0xFF0000FF, 0x00FF00FF]
    plain = archetype_from_marker(marker(Marker.LINE_STRIP, points=POINTS[:3]))
    assert len(component(plain, "strips")) == 1 and component(plain, "colors") == [0xFF8000FF]


def test_list_shapes_are_centred_on_their_points():
    boxes = archetype_from_marker(marker(Marker.CUBE_LIST, points=POINTS))
    np.testing.assert_allclose(component(boxes, "centers"), [[p.x, p.y, p.z] for p in POINTS])
    np.testing.assert_allclose(component(boxes, "half_sizes"), [[0.1, 0.2, 0.3]] * 4)


def test_triangle_lists_use_per_vertex_colours_or_the_marker_colour():
    coloured = archetype_from_marker(marker(Marker.TRIANGLE_LIST, points=POINTS[:3], colors=PER_POINT[:3]))
    assert component(coloured, "vertex_colors") == [0xFF0000FF, 0x00FF00FF, 0x0000FFFF]
    plain = archetype_from_marker(marker(Marker.TRIANGLE_LIST, points=POINTS[:3]))
    assert component(plain, "albedo_factor") == [0xFF8000FF]


def test_shapes_use_scale_as_size_and_the_marker_colour():
    cube = archetype_from_marker(marker(Marker.CUBE))
    np.testing.assert_allclose(component(cube, "half_sizes"), [[0.1, 0.2, 0.3]])
    assert component(cube, "colors") == [0xFF8000FF]
    cylinder = archetype_from_marker(marker(Marker.CYLINDER))
    assert component(cylinder, "lengths") == pytest.approx([0.6])
    assert component(cylinder, "radii") == pytest.approx([0.1])


def test_line_lists_draw_segments_coloured_by_their_first_point():
    lines = archetype_from_marker(marker(Marker.LINE_LIST, points=POINTS, colors=PER_POINT))
    assert len(component(lines, "strips")) == 2
    assert component(lines, "colors") == [0xFF0000FF, 0x0000FFFF]


def test_list_markers_use_per_point_colours():
    points = archetype_from_marker(marker(Marker.POINTS, points=POINTS, colors=PER_POINT))
    assert component(points, "colors") == [0xFF0000FF, 0x00FF00FF, 0x0000FFFF, 0x000000FF]


def test_arrows_from_pose_or_from_two_points():
    np.testing.assert_allclose(component(archetype_from_marker(marker(Marker.ARROW)), "vectors"), [[0.2, 0, 0]])
    from_points = archetype_from_marker(marker(Marker.ARROW, points=POINTS[1:3]))
    np.testing.assert_allclose(component(from_points, "vectors"), [[0, 1, 0]])


def test_text_markers_are_labels():
    text = archetype_from_marker(marker(Marker.TEXT_VIEW_FACING, text="box 3"))
    assert component(text, "labels") == ["box 3"]
    assert component(text, "show_labels") == [True]


def test_markers_are_posed_in_their_header_frame(node, build, recording):
    build("marker_array", entity_path="markers")
    m = marker(ns="boxes/left", marker_id=3)
    m.pose.position.y = 0.5
    node.publish(MarkerArray(markers=[m]))

    # The namespace's slash is escaped, so each marker stays one entity
    row = recording.rows("/markers/boxes\\/left/3")[0]
    marker_frame = "marker:markers/boxes\\/left/3"
    assert row["Transform3D:parent_frame"] == ["table"]
    assert row["Transform3D:child_frame"] == [marker_frame]
    assert row["Transform3D:translation"] == [[0.0, 0.5, 0.0]]
    assert row["CoordinateFrame:frame"] == [marker_frame]
    assert row["Boxes3D:half_sizes"] is not None


def test_markers_are_logged_at_their_stamp_or_receive_time(node, build, recording):
    build("marker_array", entity_path="markers")
    stamped = marker(marker_id=1)
    stamped.header.stamp = TimeMsg(sec=5)
    node.now_ns = 9_000_000_000
    node.publish(MarkerArray(markers=[stamped, marker(marker_id=2)]))
    assert recording.rows("/markers/ns/1")[0]["ros_time"] == 5_000_000_000
    assert recording.rows("/markers/ns/2")[0]["ros_time"] == 9_000_000_000


def test_a_marker_changing_type_loses_its_old_shape(node, build, recording):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(Marker.CUBE)]))
    node.publish(MarkerArray(markers=[marker(Marker.SPHERE)]))
    assert recording.latest("/markers/ns/0", "Boxes3D:half_sizes") == []
    assert recording.latest("/markers/ns/0", "Ellipsoids3D:half_sizes")


def test_clearing_a_marker_keeps_its_frame(node, build, recording):
    # Rerun keeps a cleared frame over a transform logged at the same time, so an rr.Clear would hide a marker
    # deleted and re-added with the same stamp
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(Marker.CUBE)]))
    node.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL), marker(Marker.SPHERE)]))
    # Including recursive clears from a parent entity
    rows = [row for entity in recording.entities() for row in recording.rows(entity)]
    assert all(row.get("Clear:is_recursive") is None for row in rows)
    assert recording.latest("/markers/ns/0", "Ellipsoids3D:half_sizes")


def clears_shape(row: dict) -> bool:
    """Whether a row empties a marker's shape: every shape component it logs is empty."""
    frame = ("Transform3D:", "CoordinateFrame:")
    logged = [value for key, value in row.items() if ":" in key and not key.startswith(frame) and value is not None]
    return bool(logged) and not any(logged)


def is_cleared(recording, entity: str) -> bool:
    return any(clears_shape(row) for row in recording.rows(entity))


def test_delete_clears_one_marker(node, build, recording):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(marker_id=1), marker(marker_id=2)]))
    node.publish(MarkerArray(markers=[Marker(action=Marker.DELETE, ns="ns", id=1)]))
    assert is_cleared(recording, "/markers/ns/1") and not is_cleared(recording, "/markers/ns/2")


def test_delete_all_with_a_namespace_clears_only_that_namespace(node, build, recording):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(ns="a"), marker(ns="b")]))
    node.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL, ns="a")]))
    assert is_cleared(recording, "/markers/a/0") and not is_cleared(recording, "/markers/b/0")


def test_delete_all_without_a_namespace_clears_every_marker(node, build, recording):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(ns="a"), marker(ns="b")]))
    node.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
    assert is_cleared(recording, "/markers/a/0") and is_cleared(recording, "/markers/b/0")


def test_markers_expire_at_the_end_of_their_lifetime(node, build, recording):
    build("marker_array", entity_path="markers")
    brief = marker(marker_id=1, lifetime=DurationMsg(sec=1))
    lasting = marker(marker_id=2, lifetime=DurationMsg(sec=100))
    node.now_ns = 10_000_000_000
    node.publish(MarkerArray(markers=[brief, lasting]))

    node.now_ns = 12_000_000_000
    node.timers[0]()

    expired = recording.rows("/markers/ns/1")
    assert clears_shape(expired[-1])
    assert expired[-1]["ros_time"] == 11_000_000_000
    assert not is_cleared(recording, "/markers/ns/2")


def test_unsupported_marker_types_warn(node, build, recording):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(Marker.ARROW_STRIP)]))
    assert node.log_messages[-1] == ("warn", f"[marker_array] marker type {Marker.ARROW_STRIP} is not supported")
    assert "/markers/ns/0" not in recording.entities()


def test_a_marker_changing_to_an_unsupported_type_loses_its_old_shape(node, build, recording):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(Marker.CUBE)]))
    node.publish(MarkerArray(markers=[marker(Marker.ARROW_STRIP)]))
    assert recording.latest("/markers/ns/0", "Boxes3D:half_sizes") == []


def test_single_marker_topics(node, build, recording):
    build("marker_array", entity_path="single", msg_type="visualization_msgs.msg.Marker")
    node.publish(marker(Marker.CYLINDER, marker_id=7))
    assert recording.rows("/single/ns/7")[0].get("Cylinders3D:lengths") is not None


def test_an_all_zero_orientation_is_treated_as_identity(node, build, recording):
    build("marker_array", entity_path="markers")
    m = marker()
    m.pose.orientation.w = 0.0
    node.publish(MarkerArray(markers=[m]))
    assert recording.rows("/markers/ns/0")[0]["Transform3D:quaternion"] == [[0.0, 0.0, 0.0, 1.0]]


def received_late(node, build):
    """A marker with a 1 s lifetime, stamped at 5 s but received at 9 s."""
    build("marker_array", entity_path="markers")
    old = marker(marker_id=1, lifetime=DurationMsg(sec=1))
    old.header.stamp = TimeMsg(sec=5)
    node.now_ns = 9_000_000_000
    node.publish(MarkerArray(markers=[old]))


def test_marker_lifetime_runs_from_when_it_is_received(node, build, recording):
    received_late(node, build)
    # Past its stamp plus lifetime, but within a second of being received
    node.now_ns = 9_500_000_000
    node.timers[0]()
    assert not is_cleared(recording, "/markers/ns/1")


def test_expired_markers_are_cleared_a_lifetime_after_their_stamp(node, build, recording):
    received_late(node, build)
    node.now_ns = 10_000_000_000
    node.timers[0]()
    cleared = [row for row in recording.rows("/markers/ns/1") if clears_shape(row)]
    assert [row["ros_time"] for row in cleared] == [6_000_000_000]


def test_empty_and_underscore_namespaces_are_distinct(node, build, recording):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(ns="", marker_id=4), marker(Marker.SPHERE, ns="_", marker_id=4)]))
    node.publish(MarkerArray(markers=[Marker(action=Marker.DELETE, ns="_", id=4)]))
    assert not is_cleared(recording, "/markers/_/4")
    assert recording.rows("/markers/_/4")[0].get("Boxes3D:half_sizes") is not None
    assert is_cleared(recording, "/markers/__/4")


def test_triangle_lists_are_scaled():
    triangles = archetype_from_marker(marker(Marker.TRIANGLE_LIST, points=POINTS[1:4]))
    # scale (0.2, 0.4, 0.6) applied to each vertex
    np.testing.assert_allclose(component(triangles, "vertex_positions"), [[0.2, 0, 0], [0.2, 0.4, 0], [0, 0.4, 0]])


def test_triangle_lists_index_every_triangle():
    # Rerun draws no triangles for an empty index list, so indices are never left to be generated
    triangles = archetype_from_marker(marker(Marker.TRIANGLE_LIST, points=POINTS + POINTS[:2]))
    assert component(triangles, "triangle_indices") == [[0, 1, 2], [3, 4, 5]]


def test_triangle_lists_switching_to_one_colour_drop_their_vertex_colours(node, build, recording):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(Marker.TRIANGLE_LIST, points=POINTS[:3], colors=PER_POINT[:3])]))
    node.publish(MarkerArray(markers=[marker(Marker.TRIANGLE_LIST, points=POINTS[:3])]))
    last = recording.rows("/markers/ns/0")[-1]
    assert not last["Mesh3D:vertex_colors"]
    assert last["Mesh3D:albedo_factor"] == [0xFF8000FF]


@pytest.fixture
def obj_mesh(tmp_path) -> Path:
    mesh = tmp_path / "triangle.obj"
    mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    return mesh


def test_mesh_markers_are_drawn_from_their_file_scaled_and_in_the_marker_colour(node, build, recording, obj_mesh):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, mesh_resource=str(obj_mesh))]))

    row = recording.rows("/markers/ns/0")[0]
    assert bytes(row["Asset3D:blob"][0]) == obj_mesh.read_bytes()
    assert row["Asset3D:media_type"] == ["model/obj"]
    assert row["Asset3D:albedo_factor"] == [0xFF8000FF]
    assert row["Transform3D:scale"] == [pytest.approx([0.2, 0.4, 0.6])]


def test_collada_mesh_markers_are_scaled_by_their_unit(node, build, recording):
    build("marker_array", entity_path="markers")
    mesh = DATA_DIR / "triangle_inches.dae"
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, mesh_resource=f"file://{mesh}")]))

    row = recording.rows("/markers/ns/0")[0]
    # Rerun's SDK does not guess Collada's media type, but its viewer loads it
    assert row["Asset3D:media_type"] == ["model/vnd.collada+xml"]
    assert row["Transform3D:scale"] == [pytest.approx([0.2 * 0.0254, 0.4 * 0.0254, 0.6 * 0.0254])]


def test_mesh_markers_with_embedded_materials_keep_their_colours(node, build, recording, obj_mesh):
    build("marker_array", entity_path="markers")
    mesh = marker(Marker.MESH_RESOURCE, mesh_resource=str(obj_mesh), mesh_use_embedded_materials=True)
    node.publish(MarkerArray(markers=[mesh]))
    assert recording.rows("/markers/ns/0")[0]["Asset3D:albedo_factor"] == []


def test_mesh_markers_switching_to_embedded_materials_drop_their_colour(node, build, recording, obj_mesh):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, mesh_resource=str(obj_mesh))]))
    mesh = marker(Marker.MESH_RESOURCE, mesh_resource=str(obj_mesh), mesh_use_embedded_materials=True)
    node.publish(MarkerArray(markers=[mesh]))
    assert recording.latest("/markers/ns/0", "Asset3D:albedo_factor") == []


def test_unchanged_meshes_are_logged_once(node, build, recording, obj_mesh):
    build("marker_array", entity_path="markers")
    for colour in (ColorRGBA(r=1.0, a=1.0), ColorRGBA(r=1.0, a=1.0), ColorRGBA(g=1.0, a=1.0)):
        mesh = marker(Marker.MESH_RESOURCE, mesh_resource=str(obj_mesh))
        mesh.color = colour
        node.publish(MarkerArray(markers=[mesh]))

    rows = recording.rows("/markers/ns/0")
    assert sum("Transform3D:translation" in row for row in rows) == 3
    # Logged again only when its colour changed
    assert [row["Asset3D:albedo_factor"] for row in rows if "Asset3D:blob" in row] == [[0xFF0000FF], [0x00FF00FF]]


@pytest.mark.parametrize(
    ("resource", "error"),
    [
        ("/does/not/exist.stl", "mesh '/does/not/exist.stl' not found"),
        ("package://not_a_package/meshes/x.stl", "mesh 'package://not_a_package/meshes/x.stl' not found"),
        ("http://example.com/x.stl", "mesh 'http://example.com/x.stl' is not a package:// or file:// path"),
    ],
)
def test_meshes_that_cannot_be_found_warn_and_clear_the_marker(node, build, recording, resource, error):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(Marker.CUBE)]))
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, mesh_resource=resource)]))
    assert node.log_messages[-1] == ("warn", f"[marker_array] {error}")
    assert recording.latest("/markers/ns/0", "Boxes3D:half_sizes") == []


def test_meshes_in_formats_rerun_does_not_load_warn(node, build, tmp_path):
    build("marker_array", entity_path="markers")
    mesh = tmp_path / "part.3ds"
    mesh.write_bytes(b"")
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, mesh_resource=str(mesh))]))
    assert "is not in a format Rerun loads" in node.log_messages[-1][1]


def test_collada_versions_rerun_does_not_load_warn(node, build, tmp_path):
    build("marker_array", entity_path="markers")
    mesh = tmp_path / "part.dae"
    mesh.write_text('<?xml version="1.0"?>\n<COLLADA version="1.4.0"><asset/></COLLADA>')
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, mesh_resource=str(mesh))]))
    assert node.log_messages[-1][1] == f"[marker_array] mesh '{mesh}' is Collada 1.4.0, and Rerun only loads 1.4.1"


def test_a_mesh_marker_changing_type_loses_its_mesh_and_scale(node, build, recording, obj_mesh):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, mesh_resource=str(obj_mesh))]))
    node.publish(MarkerArray(markers=[marker(Marker.CUBE)]))
    assert recording.latest("/markers/ns/0", "Asset3D:blob") == []
    assert recording.latest("/markers/ns/0", "Transform3D:scale") == []


def test_a_deleted_mesh_marker_re_added_unchanged_is_drawn_again(node, build, recording, obj_mesh):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, mesh_resource=str(obj_mesh))]))
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, action=Marker.DELETE)]))
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, mesh_resource=str(obj_mesh))]))
    assert bytes(recording.latest("/markers/ns/0", "Asset3D:blob")[0]) == obj_mesh.read_bytes()


def mesh_at(resource: Path, seconds: int, colour: ColorRGBA) -> MarkerArray:
    mesh = marker(Marker.MESH_RESOURCE, mesh_resource=str(resource))
    mesh.header.stamp = TimeMsg(sec=seconds)
    mesh.color = colour
    return MarkerArray(markers=[mesh])


def test_late_mesh_markers_are_drawn_in_their_place(node, build, recording, obj_mesh):
    red, green = ColorRGBA(r=1.0, a=1.0), ColorRGBA(g=1.0, a=1.0)
    build("marker_array", entity_path="markers")
    node.publish(mesh_at(obj_mesh, 10, red))
    node.publish(mesh_at(obj_mesh, 20, red))
    # Received last, e.g. from a replayed bag
    node.publish(mesh_at(obj_mesh, 15, green))

    colours = sorted(
        (row["ros_time"], row["Asset3D:albedo_factor"][0])
        for row in recording.rows("/markers/ns/0")
        if "Asset3D:albedo_factor" in row
    )
    # Red from 10 s, green from 15 s, and red again from 20 s, as the meshes were published
    assert colours == [(10_000_000_000, 0xFF0000FF), (15_000_000_000, 0x00FF00FF), (20_000_000_000, 0xFF0000FF)]


def test_files_that_are_not_meshes_warn(node, build, tmp_path):
    build("marker_array", entity_path="markers")
    image = tmp_path / "part.png"
    image.write_bytes(b"")
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, mesh_resource=str(image))]))
    assert "is not in a format Rerun loads" in node.log_messages[-1][1]


def test_gltf_meshes_referring_to_other_files_warn(node, build, recording, tmp_path):
    build("marker_array", entity_path="markers")
    mesh = tmp_path / "part.gltf"
    mesh.write_text('{"buffers": [{"uri": "part.bin", "byteLength": 36}]}')
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, mesh_resource=str(mesh))]))
    assert (
        node.log_messages[-1][1]
        == f"[marker_array] mesh '{mesh}' refers to other files, which Rerun cannot load: use a .glb file"
    )


def test_gltf_meshes_with_embedded_buffers_are_drawn(node, build, recording, tmp_path):
    build("marker_array", entity_path="markers")
    mesh = tmp_path / "part.gltf"
    mesh.write_text('{"buffers": [{"uri": "data:application/octet-stream;base64,AAAA", "byteLength": 3}]}')
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, mesh_resource=str(mesh))]))
    assert recording.rows("/markers/ns/0")[0]["Asset3D:media_type"] == ["model/gltf+json"]


def test_file_uris_are_decoded(node, build, recording, tmp_path):
    build("marker_array", entity_path="markers")
    mesh = tmp_path / "my parts" / "triangle.obj"
    mesh.parent.mkdir()
    mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    uris = [mesh.as_uri(), f"file://localhost{mesh.as_uri().removeprefix('file://')}"]
    meshes = [marker(Marker.MESH_RESOURCE, marker_id=i, mesh_resource=uri) for i, uri in enumerate(uris)]
    node.publish(MarkerArray(markers=meshes))
    for i in range(len(uris)):
        assert bytes(recording.latest(f"/markers/ns/{i}", "Asset3D:blob")[0]) == mesh.read_bytes()


def test_meshes_that_cannot_be_read_warn_and_clear_the_marker(node, build, recording, obj_mesh, monkeypatch):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(Marker.CUBE)]))

    # A file found but not readable, which a permission check would not show when the tests run as root, e.g. in CI
    def unreadable(path: Path) -> bytes:
        raise PermissionError(13, "Permission denied", str(path))

    monkeypatch.setattr(Path, "read_bytes", unreadable)
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE, mesh_resource=str(obj_mesh))]))
    assert "Permission denied" in node.log_messages[-1][1]
    assert recording.latest("/markers/ns/0", "Boxes3D:half_sizes") == []
