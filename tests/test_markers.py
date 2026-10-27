import numpy as np
import pytest
from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import rerun as rr

from ros_rerun_bridge.modules.marker_array import archetype_from_marker

POINTS = [
    Point(x=0.0, y=0.0, z=0.0),
    Point(x=1.0, y=0.0, z=0.0),
    Point(x=1.0, y=1.0, z=0.0),
    Point(x=0.0, y=1.0, z=0.0),
]
PER_POINT = [ColorRGBA(r=1.0, a=1.0), ColorRGBA(g=1.0, a=1.0), ColorRGBA(b=1.0, a=1.0), ColorRGBA(a=1.0)]


def marker(marker_type: int = Marker.CUBE, ns: str = "ns", marker_id: int = 0, **fields) -> Marker:
    m = Marker(type=marker_type, ns=ns, id=marker_id, **fields)
    m.header.frame_id = "conveyor"
    m.scale.x, m.scale.y, m.scale.z = 0.2, 0.4, 0.6
    m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
    m.pose.orientation.w = 1.0
    return m


def component(archetype, name: str):
    return getattr(archetype, name).as_arrow_array().to_pylist()


@pytest.mark.parametrize(
    ("marker_type", "archetype"),
    [
        (Marker.ARROW, rr.Arrows3D),
        (Marker.CUBE, rr.Boxes3D),
        (Marker.SPHERE, rr.Ellipsoids3D),
        (Marker.CYLINDER, rr.Cylinders3D),
        (Marker.LINE_STRIP, rr.LineStrips3D),
        (Marker.LINE_LIST, rr.LineStrips3D),
        (Marker.CUBE_LIST, rr.Boxes3D),
        (Marker.SPHERE_LIST, rr.Ellipsoids3D),
        (Marker.POINTS, rr.Points3D),
        (Marker.TEXT_VIEW_FACING, rr.Points3D),
        (Marker.TRIANGLE_LIST, rr.Mesh3D),
    ],
)
def test_every_marker_type_but_mesh_resource_converts(marker_type, archetype):
    assert isinstance(archetype_from_marker(marker(marker_type, points=POINTS[:3], text="t")), archetype)


def test_mesh_resource_markers_are_not_supported():
    assert archetype_from_marker(marker(Marker.MESH_RESOURCE)) is None


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
    text = archetype_from_marker(marker(Marker.TEXT_VIEW_FACING, text="tray 3"))
    assert component(text, "labels") == ["tray 3"]
    assert component(text, "show_labels") == [True]


def test_markers_are_posed_in_their_header_frame(node, build, recording):
    build("marker_array", entity_path="markers")
    m = marker(ns="trays/inlet", marker_id=3)
    m.pose.position.y = 0.5
    node.publish(MarkerArray(markers=[m]))

    # The namespace's slash is escaped, so each marker stays one entity
    row = recording.rows("/markers/trays\\/inlet/3")[0]
    marker_frame = "marker:markers/trays\\/inlet/3"
    assert row["Transform3D:parent_frame"] == ["conveyor"]
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
    rows = recording.rows("/markers/ns/0")
    assert "Clear:is_recursive" in rows[1] and rows[2].get("Ellipsoids3D:half_sizes") is not None


def is_cleared(recording, entity: str) -> bool:
    return any(row.get("Clear:is_recursive") is not None for row in recording.rows(entity))


def test_delete_clears_one_marker(node, build, recording):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(marker_id=1), marker(marker_id=2)]))
    node.publish(MarkerArray(markers=[Marker(action=Marker.DELETE, ns="ns", id=1)]))
    assert is_cleared(recording, "/markers/ns/1") and not is_cleared(recording, "/markers/ns/2")


def test_delete_all_with_a_namespace_clears_only_that_namespace(node, build, recording):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(ns="a"), marker(ns="b")]))
    node.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL, ns="a")]))
    assert is_cleared(recording, "/markers/a") and not is_cleared(recording, "/markers")


def test_delete_all_without_a_namespace_clears_every_marker(node, build, recording):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(ns="a"), marker(ns="b")]))
    node.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
    assert is_cleared(recording, "/markers")


def test_markers_expire_at_the_end_of_their_lifetime(node, build, recording):
    build("marker_array", entity_path="markers")
    brief = marker(marker_id=1, lifetime=DurationMsg(sec=1))
    lasting = marker(marker_id=2, lifetime=DurationMsg(sec=100))
    node.now_ns = 10_000_000_000
    node.publish(MarkerArray(markers=[brief, lasting]))

    node.now_ns = 12_000_000_000
    node.timers[0]()

    expired = recording.rows("/markers/ns/1")
    assert expired[-1].get("Clear:is_recursive") is not None
    assert expired[-1]["ros_time"] == 11_000_000_000
    assert not is_cleared(recording, "/markers/ns/2")


def test_unsupported_marker_types_warn(node, build, recording):
    build("marker_array", entity_path="markers")
    node.publish(MarkerArray(markers=[marker(Marker.MESH_RESOURCE)]))
    assert node.log_messages[-1] == ("warn", f"[marker_array] marker type {Marker.MESH_RESOURCE} is not supported")
    assert "/markers/ns/0" not in recording.entities()


def test_single_marker_topics(node, build, recording):
    build("marker_array", entity_path="single", msg_type="visualization_msgs.msg.Marker")
    node.publish(marker(Marker.CYLINDER, marker_id=7))
    assert recording.rows("/single/ns/7")[0].get("Cylinders3D:lengths") is not None
