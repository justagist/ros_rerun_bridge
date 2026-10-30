"""Numeric message fields plotted as time series."""

import math

import pytest
from builtin_interfaces.msg import Time as TimeMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState, Range
from std_msgs.msg import Float64

import rerun as rr

from ros_rerun_bridge.utils.fields import is_string_array


def build_scalars(build, msg_type: str, *fields: str):
    return build("scalars", entity_path="plots", msg_type=msg_type, params={"fields": list(fields)})


def scalars(recording, entity: str) -> list:
    return [row["Scalars:scalars"] for row in recording.rows(entity) if not row["static"]]


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
            "width": 2.0,
            "interpolation": "step_after",
        },
    )
    node.publish(JointState(position=[0.1, 0.2]))
    node.publish(JointState(position=[0.3, 0.4]))

    [style] = static_rows(recording, "/plots/position")
    assert style["SeriesLines:names"] == ["shoulder", "elbow"]
    assert style["SeriesLines:widths"] == [2.0]
    assert style["SeriesLines:interpolation_mode"] == [rr.components.InterpolationMode.StepAfter.value]


def test_point_style_draws_points(build, recording):
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
    ],
)
def test_invalid_style_is_an_error(build, params, error):
    with pytest.raises(ValueError, match=error):
        build("scalars", msg_type="sensor_msgs.msg.Range", params={"fields": ["range"], **params})


def test_variance_is_logged_with_each_value_as_a_measurement(node, build, recording):
    build(
        "scalars",
        entity_path="plots",
        msg_type="sensor_msgs.msg.Range",
        params={"fields": ["range"], "variance": "variance"},
    )
    node.publish(Range(range=1.5, variance=0.04))
    [row] = [row for row in recording.rows("/plots/range") if not row["static"]]
    assert row["Measurements:values"] == [1.5]
    assert row["Measurements:variances"] == pytest.approx([0.04])
    assert "Scalars:scalars" not in row


def test_variance_applies_to_array_elements_and_pads_shrinking_arrays(node, build, recording):
    build(
        "scalars",
        entity_path="plots",
        msg_type="sensor_msgs.msg.BatteryState",
        params={"fields": ["cell_voltage"], "variance": "voltage"},
    )
    node.publish(BatteryState(cell_voltage=[3.7, 3.8], voltage=0.01))
    node.publish(BatteryState(cell_voltage=[3.9], voltage=0.02))
    [first, second] = [row for row in recording.rows("/plots/cell_voltage") if not row["static"]]
    assert first["Measurements:variances"] == pytest.approx([0.01, 0.01])
    assert second["Measurements:values"] == pytest.approx([3.9, math.nan], nan_ok=True)
    assert second["Measurements:variances"] == pytest.approx([0.02, math.nan], nan_ok=True)


def test_measurement_unit_and_style_are_logged_once_as_static(node, build, recording):
    build(
        "scalars",
        entity_path="plots",
        msg_type="sensor_msgs.msg.Range",
        params={"fields": ["range"], "unit": "m", "names": ["distance"]},
    )
    node.publish(Range(range=1.5))
    [style] = static_rows(recording, "/plots/range")
    assert style["Measurements:units"] == ["m"]
    assert style["Measurements:names"] == ["distance"]
    # A unit alone also makes the values measurements
    [row] = [row for row in recording.rows("/plots/range") if not row["static"]]
    assert row["Measurements:values"] == [1.5]


@pytest.mark.parametrize(
    ("params", "error"),
    [
        ({"fields": ["range"], "variance": "variance", "style": "points"}, "cannot be used with style 'points'"),
        ({"fields": ["range", "max_range"], "variance": "variance"}, "variance needs exactly one field"),
        ({"fields": ["range"], "variance": "header"}, "is std_msgs/Header, not a number"),
    ],
)
def test_invalid_measurement_params_are_an_error(build, params, error):
    with pytest.raises(ValueError, match=error):
        build("scalars", msg_type="sensor_msgs.msg.Range", params=params)


def test_an_array_of_variances_is_an_error(build):
    # Its length could differ from the values' on any message
    with pytest.raises(ValueError, match="variance 'd' is sequence<double>, not a single number"):
        build("scalars", msg_type="sensor_msgs.msg.CameraInfo", params={"fields": ["k"], "variance": "d"})


def build_named(build, *fields: str, **params):
    return build(
        "scalars",
        entity_path="plots",
        msg_type="sensor_msgs.msg.JointState",
        params={"fields": list(fields), "names_field": "name", **params},
    )


def test_named_elements_keep_their_series_whichever_publisher_sends_them(node, build, recording):
    build_named(build, "position")
    # As on /joint_states, with an arm and a gate published by different nodes
    node.publish(JointState(name=["shoulder", "elbow"], position=[0.1, 0.2]))
    node.publish(JointState(name=["gate"], position=[0.9]))
    node.publish(JointState(name=["elbow", "shoulder"], position=[0.4, 0.3]))

    assert scalars(recording, "/plots/position/shoulder") == [[0.1], [0.3]]
    assert scalars(recording, "/plots/position/elbow") == [[0.2], [0.4]]
    assert scalars(recording, "/plots/position/gate") == [[0.9]]


def test_named_series_are_named_after_their_element(node, build, recording):
    build_named(build, "position", width=2.0)
    node.publish(JointState(name=["shoulder"], position=[0.1]))
    node.publish(JointState(name=["shoulder"], position=[0.2]))
    [style] = static_rows(recording, "/plots/position/shoulder")
    assert style["SeriesLines:names"] == ["shoulder"]
    assert style["SeriesLines:widths"] == [2.0]


def test_empty_named_fields_are_skipped(node, build, recording):
    build_named(build, "position", "velocity")
    node.publish(JointState(name=["shoulder"], position=[0.1]))
    assert scalars(recording, "/plots/position/shoulder") == [[0.1]]
    assert not any(entity.startswith("/plots/velocity") for entity in recording.entities())
    # Not an error, as the JointState spec allows it
    assert not [message for level, message in node.log_messages if level == "error"]


def test_named_fields_with_as_many_values_as_names_are_required(node, build):
    build_named(build, "position")
    node.publish(JointState(name=["shoulder", "elbow"], position=[0.1]))
    assert "'position' has 1 values for the 2 names in 'name'" in node.log_messages[-2][1]


def test_named_elements_can_be_measurements(node, build, recording):
    build_named(build, "position", unit="rad")
    node.publish(JointState(name=["shoulder"], position=[0.1]))
    [row] = [row for row in recording.rows("/plots/position/shoulder") if not row["static"]]
    assert row["Measurements:values"] == [0.1]
    [style] = static_rows(recording, "/plots/position/shoulder")
    assert style["Measurements:units"] == ["rad"]


@pytest.mark.parametrize(
    ("msg_type", "params", "error"),
    [
        ("sensor_msgs.msg.JointState", {"fields": ["position"], "names_field": "velocity"}, "not an array of strings"),
        ("sensor_msgs.msg.JointState", {"fields": ["position"], "names_field": "header"}, "not an array of strings"),
        ("sensor_msgs.msg.JointState", {"fields": ["position"], "names_field": "name", "names": ["a"]}, "cannot both"),
        ("sensor_msgs.msg.Range", {"fields": ["range"], "names_field": "header.frame_id"}, "not an array of strings"),
        (
            "sensor_msgs.msg.JointState",
            {"fields": ["position", "header.stamp.sec"], "names_field": "name"},
            "header.stamp.sec is not an array",
        ),
    ],
)
def test_invalid_names_field_is_an_error(build, msg_type, params, error):
    with pytest.raises(ValueError, match=error):
        build("scalars", msg_type=msg_type, params=params)


@pytest.mark.parametrize(
    ("ros_type", "expected"),
    [
        ("sequence<string>", True),
        ("string[3]", True),
        # As ROS spells arrays of bounded and wide strings, e.g. test_msgs/Strings.bounded_string_value is string<22>
        ("sequence<string<22>>", True),
        ("sequence<string<22>, 5>", True),
        ("wstring<5>[3]", True),
        ("string<22>", False),
        ("sequence<double>", False),
    ],
)
def test_names_field_can_be_any_array_of_strings(ros_type, expected):
    assert is_string_array(ros_type) == expected
