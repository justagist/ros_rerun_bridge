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
