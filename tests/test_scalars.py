"""Numeric message fields plotted as time series."""

import math

import pytest
from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float64

import rerun as rr


def build_scalars(build, msg_type: str, *fields: str):
    return build("scalars", entity_path="plots", msg_type=msg_type, params={"fields": list(fields)})


def scalars(recording, entity: str) -> list:
    return [row["Scalars:scalars"] for row in recording.rows(entity)]


def test_each_field_is_plotted_on_its_own_entity(node, build, recording):
    build_scalars(build, "sensor_msgs.msg.Range", "range", "max_range")
    node.publish(Range(range=0.5, max_range=2.0))
    node.publish(Range(range=0.75, max_range=2.0))
    assert scalars(recording, "/plots/range") == [[0.5], [0.75]]
    assert scalars(recording, "/plots/max_range") == [[2.0], [2.0]]


def test_nested_fields_are_plotted_under_their_path(node, build, recording):
    build_scalars(build, "nav_msgs.msg.Odometry", "twist.twist.linear.x")
    msg = Odometry()
    msg.twist.twist.linear.x = 1.5
    node.publish(msg)
    assert scalars(recording, "/plots/twist/twist/linear/x") == [[1.5]]


def test_arrays_are_plotted_as_one_series_per_element(node, build, recording):
    build_scalars(build, "sensor_msgs.msg.JointState", "position")
    node.publish(JointState(position=[0.1, 0.2, 0.3]))
    assert scalars(recording, "/plots/position") == [[0.1, 0.2, 0.3]]


def test_shorter_arrays_are_padded_to_leave_gaps_in_the_series_they_lack(node, build, recording):
    build_scalars(build, "sensor_msgs.msg.JointState", "position")
    node.publish(JointState(position=[0.1, 0.2, 0.3]))
    node.publish(JointState(position=[0.4]))
    assert scalars(recording, "/plots/position")[-1] == pytest.approx([0.4, math.nan, math.nan], nan_ok=True)


def test_non_finite_values_are_kept_to_break_the_line(node, build, recording):
    build_scalars(build, "sensor_msgs.msg.Range", "range")
    node.publish(Range(range=math.inf))
    assert scalars(recording, "/plots/range") == [[math.inf]]


def test_values_are_logged_at_their_stamp_or_receive_time(node, build, recording):
    build_scalars(build, "sensor_msgs.msg.Range", "range")
    stamped = Range(range=1.0)
    stamped.header.stamp = TimeMsg(sec=5)
    node.publish(stamped)
    build_scalars(build, "std_msgs.msg.Float64", "data")
    node.now_ns = 9_000_000_000
    node.publish(Float64(data=2.0))

    assert recording.rows("/plots/range")[0]["ros_time"] == 5_000_000_000
    assert recording.rows("/plots/data")[0]["ros_time"] == 9_000_000_000


@pytest.mark.parametrize(
    ("msg_type", "fields", "error"),
    [
        (None, ["range"], "needs `msg_type`"),
        ("sensor_msgs.msg.Range", [], "fields"),
        ("sensor_msgs.msg.Range", ["distance"], "Range has no field 'distance'"),
        ("sensor_msgs.msg.Range", ["header"], "is std_msgs/Header, not a number"),
        ("sensor_msgs.msg.JointState", ["name"], "is sequence<string>, not a number"),
        ("std_msgs.msg.Byte", ["data"], "is octet, not a number"),
        ("sensor_msgs.msg.Range", ["range.x"], "'range' is not a message"),
        ("visualization_msgs.msg.MarkerArray", ["markers.id"], "'markers' is not a message"),
    ],
)
def test_invalid_config_is_an_error(build, msg_type, fields, error):
    with pytest.raises(ValueError, match=error):
        build("scalars", msg_type=msg_type, params={"fields": fields})


def static_rows(recording, entity: str) -> list[dict]:
    return [row for row in recording.rows(entity) if row["static"]]


def test_line_style_is_logged_once_as_static(node, build, recording):
    build(
        "scalars",
        entity_path="plots",
        msg_type="sensor_msgs.msg.JointState",
        params={
            "fields": ["position"],
            "names": ["shoulder", "elbow"],
            "colours": [[255, 0, 0, 255]],
            "width": 2.0,
            "interpolation": "step_after",
        },
    )
    node.publish(JointState(position=[0.1, 0.2]))
    node.publish(JointState(position=[0.3, 0.4]))

    [style] = static_rows(recording, "/plots/position")
    assert style["SeriesLines:names"] == ["shoulder", "elbow"]
    assert style["SeriesLines:colors"] == [0xFF0000FF]
    assert style["SeriesLines:widths"] == [2.0]
    assert style["SeriesLines:interpolation_mode"] == [rr.components.InterpolationMode.StepAfter.value]


def test_point_style_draws_points(node, build, recording):
    build(
        "scalars",
        entity_path="plots",
        msg_type="sensor_msgs.msg.Range",
        params={"fields": ["range"], "style": "points", "marker_size": 3.0},
    )
    [style] = static_rows(recording, "/plots/range")
    # A SeriesPoints component on the entity is what makes Rerun draw points
    assert style["SeriesPoints:markers"] == [rr.components.MarkerShape.Circle.value]
    assert style["SeriesPoints:marker_sizes"] == [3.0]
    assert not any(key.startswith("SeriesLines:") for key in style)


def test_without_style_params_rerun_default_lines_are_kept(node, build, recording):
    build_scalars(build, "sensor_msgs.msg.Range", "range")
    node.publish(Range(range=0.5))
    assert static_rows(recording, "/plots/range") == []


@pytest.mark.parametrize(
    ("params", "error"),
    [
        ({"marker": "square"}, "marker cannot be used with style 'lines'"),
        ({"style": "points", "width": 2.0, "interpolation": "step_mid"}, "width, interpolation cannot be used"),
        ({"interpolation": "steps"}, "interpolation"),
        ({"colours": [[256, 0, 0, 255]]}, "less than or equal to 255"),
        ({"colours": [[-1, 0, 0, 255]]}, "greater than or equal to 0"),
    ],
)
def test_invalid_style_is_an_error(build, params, error):
    with pytest.raises(ValueError, match=error):
        build("scalars", msg_type="sensor_msgs.msg.Range", params={"fields": ["range"], **params})
