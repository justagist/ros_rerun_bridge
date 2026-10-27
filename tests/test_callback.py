"""Behaviour every module gets from the base class: message time, frame placement and error handling."""

from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import Image, Range
from std_msgs.msg import String


def reading(sec: int = 0, frame_id: str = "sensor") -> Range:
    msg = Range(field_of_view=0.5, min_range=0.1, max_range=2.0, range=1.0)
    msg.header.stamp, msg.header.frame_id = TimeMsg(sec=sec), frame_id
    return msg


def test_data_is_logged_at_the_message_stamp(node, build, recording):
    build("range", entity_path="range")
    node.publish(reading(sec=7))
    assert recording.rows("/range")[0]["ros_time"] == 7_000_000_000


def test_unstamped_messages_are_logged_at_receive_time(node, build, recording):
    build("range", entity_path="range")
    node.now_ns = 42_000_000_000
    node.publish(reading(sec=0))
    assert recording.rows("/range")[0]["ros_time"] == 42_000_000_000


def test_data_is_placed_in_the_header_frame(node, build, recording):
    build("range", entity_path="range")
    node.publish(reading(frame_id="inlet_sensor"))
    assert recording.rows("/range")[0]["CoordinateFrame:frame"] == ["inlet_sensor"]


def test_handler_errors_are_logged_with_their_type_not_raised(node, build):
    build("image", entity_path="image")
    node.publish(Image(height=1, width=1, encoding="nv21", step=1, data=b"\x00"))
    assert node.log_messages[-2] == (
        "error",
        "[image] error handling a message: ValueError: unsupported image encoding 'nv21'",
    )
    assert node.log_messages[-1][0] == "debug" and "Traceback" in node.log_messages[-1][1]


def test_errors_reading_the_stamp_are_logged_not_raised(node, build):
    # A message type without a header, as a msg_type override could give
    build("range", entity_path="range", msg_type="std_msgs.msg.String")
    node.publish(String(data="no header"))
    assert node.log_messages[-2][0] == "error" and "AttributeError" in node.log_messages[-2][1]
