import pytest
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


def transform(parent: str, child: str, x: float, sec: int = 0) -> TransformStamped:
    tf = TransformStamped(child_frame_id=child)
    tf.header.frame_id, tf.header.stamp = parent, TimeMsg(sec=sec)
    tf.transform.translation.x = x
    tf.transform.rotation.w = 1.0
    return tf


def test_logs_each_transform_between_named_frames_at_its_own_stamp(node, build, recording):
    build("tf", entity_path="tf")
    node.publish(TFMessage(transforms=[transform("map", "odom", 1.0, sec=3), transform("odom", "base", 2.0, sec=4)]))

    rows = recording.rows("/tf")
    assert [(r["Transform3D:parent_frame"], r["Transform3D:child_frame"]) for r in rows] == [
        (["map"], ["odom"]),
        (["odom"], ["base"]),
    ]
    assert [r["Transform3D:translation"] for r in rows] == [[[1.0, 0.0, 0.0]], [[2.0, 0.0, 0.0]]]
    assert [r["ros_time"] for r in rows] == [3_000_000_000, 4_000_000_000]


def test_static_transforms_are_logged_as_static(node, build, recording):
    build("tf", entity_path="tf_static", params={"static": True})
    node.publish(TFMessage(transforms=[transform("map", "camera", 0.5)]))

    rows = recording.rows("/tf_static")
    assert len(rows) == 1 and rows[0]["static"]
    assert rows[0]["Transform3D:child_frame"] == ["camera"]


def test_unstamped_transforms_are_logged_at_receive_time(node, build, recording):
    build("tf", entity_path="tf")
    node.now_ns = 42_000_000_000
    node.publish(TFMessage(transforms=[transform("map", "odom", 1.0, sec=0)]))
    assert recording.rows("/tf")[0]["ros_time"] == 42_000_000_000


def test_each_frame_gets_its_own_axes_entity_once(node, build, recording):
    # One entity per frame, so the viewer can show or hide each frame's axes and name
    build("tf", entity_path="tf", params={"axis_length": 0.3, "frame_names": True})
    node.publish(TFMessage(transforms=[transform("map", "odom", 1.0), transform("odom", "base/link", 2.0)]))
    node.publish(TFMessage(transforms=[transform("odom", "base/link", 3.0)]))

    frames = sorted(entity for entity in recording.entities() if entity.startswith("/frames/"))
    assert frames == ["/frames/base\\/link", "/frames/map", "/frames/odom"]
    [axes] = recording.rows("/frames/base\\/link")
    assert axes["static"]
    assert axes["CoordinateFrame:frame"] == ["base/link"]
    assert axes["TransformAxes3D:axis_length"] == pytest.approx([0.3])
    assert axes["TransformAxes3D:show_frame"] == [True]


def test_without_axis_length_no_axes_are_drawn(node, build, recording):
    build("tf", entity_path="tf")
    node.publish(TFMessage(transforms=[transform("map", "odom", 1.0)]))
    rows = [row for entity in recording.entities() for row in recording.rows(entity)]
    assert not [row for row in rows if any(key.startswith("TransformAxes3D:") for key in row)]


def test_frame_names_need_axes(build):
    with pytest.raises(ValueError, match="frame_names needs axis_length"):
        build("tf", params={"frame_names": True})


@pytest.mark.parametrize("axis_length", [0.0, -0.1, float("inf"), float("nan")])
def test_axis_length_must_be_finite_and_positive(build, axis_length):
    with pytest.raises(ValueError, match="axis_length"):
        build("tf", params={"axis_length": axis_length})


def test_a_frame_seen_by_two_modules_has_one_entity(node, build, recording):
    # E.g. a robot's base, the child of a static transform and the parent of its first joint
    build("tf", entity_path="tf_static", params={"static": True, "axis_length": 0.3})
    node.publish(TFMessage(transforms=[transform("world", "base", 1.0)]), index=0)
    build("tf", entity_path="tf", params={"axis_length": 0.3})
    node.publish(TFMessage(transforms=[transform("base", "arm", 1.0)]), index=1)
    frames = sorted(entity for entity in recording.entities() if entity.startswith("/frames/"))
    assert frames == ["/frames/arm", "/frames/base", "/frames/world"]
