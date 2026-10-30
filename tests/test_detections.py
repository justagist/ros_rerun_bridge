"""2D and 3D detections."""

import math
import zlib

import numpy as np
import pytest
from vision_msgs.msg import Detection2D, Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose


def hypothesis(class_id: str, score: float) -> ObjectHypothesisWithPose:
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id, result.hypothesis.score = class_id, score
    return result


def box_2d(x: float, y: float, width: float, height: float, theta: float = 0.0, **fields) -> Detection2D:
    detection = Detection2D(**fields)
    # float64 fields only take floats when ROS checks message fields (ROS_PYTHON_CHECK_FIELDS)
    detection.bbox.center.position.x, detection.bbox.center.position.y = float(x), float(y)
    detection.bbox.center.theta = float(theta)
    detection.bbox.size_x, detection.bbox.size_y = float(width), float(height)
    return detection


def detections_2d(*detections: Detection2D) -> Detection2DArray:
    msg = Detection2DArray(detections=list(detections))
    msg.header.frame_id = "camera"
    return msg


def test_2d_boxes_are_closed_strips_on_the_image_plane(node, build, recording):
    build("detections_2d", entity_path="camera/detections")
    node.publish(detections_2d(box_2d(10, 20, 4, 2), box_2d(50, 50, 4, 2, theta=math.pi / 2)))

    row = recording.rows("/camera/detections")[0]
    assert row["CoordinateFrame:frame"] == ["camera_image_plane"]
    upright, rotated = row["LineStrips2D:strips"]
    # From the top left, round to it again
    np.testing.assert_allclose(upright, [[8, 19], [12, 19], [12, 21], [8, 21], [8, 19]])
    np.testing.assert_allclose(rotated, [[51, 48], [51, 52], [49, 52], [49, 48], [51, 48]], atol=1e-9)


@pytest.mark.parametrize(
    ("results", "tracking_id", "label"),
    [
        ([hypothesis("cat", 0.3), hypothesis("person", 0.92)], "7", "person #7 0.92"),
        ([hypothesis("person", 0.92)], "", "person 0.92"),
        ([], "7", "#7"),
    ],
)
def test_detections_are_labelled_with_their_best_class_tracking_id_and_score(
    node, build, recording, results, tracking_id, label
):
    build("detections_2d", entity_path="camera/detections")
    node.publish(detections_2d(box_2d(10, 20, 4, 2, results=results, id=tracking_id)))
    assert recording.rows("/camera/detections")[0]["LineStrips2D:labels"] == [label]


def test_listed_classes_have_their_labels_and_colours(node, build, recording):
    classes = [{"id": "1", "label": "person", "colour": [220, 40, 40, 255]}, {"id": "2"}]
    build("detections_2d", entity_path="camera/detections", params={"classes": classes})
    node.publish(
        detections_2d(
            box_2d(10, 20, 4, 2, results=[hypothesis("2", 0.5)]), box_2d(30, 20, 4, 2, results=[hypothesis("1", 0.8)])
        )
    )

    row = recording.rows("/camera/detections")[-1]
    assert row["LineStrips2D:labels"] == ["2 0.50", "person 0.80"]
    assert row["LineStrips2D:class_ids"] == [1, 0]
    [context] = recording.latest("/camera/detections", "AnnotationContext:context")
    assert [c["class_description"]["info"]["color"] for c in context] == [0xDC2828FF, None]


def test_classes_not_listed_keep_one_class_id_above_the_listed_ones(node, build, recording):
    build("detections_2d", entity_path="camera/detections", params={"classes": [{"id": "person"}]})
    for _ in range(2):
        node.publish(detections_2d(box_2d(10, 20, 4, 2, results=[hypothesis("dog", 0.5)])))

    class_ids = [
        row["LineStrips2D:class_ids"] for row in recording.rows("/camera/detections") if "LineStrips2D:class_ids" in row
    ]
    # A checksum, unlike Python's hash, is the same in every run
    assert class_ids == [[1 + zlib.crc32(b"dog") % 65535]] * 2


def test_no_detections_clear_the_boxes(node, build, recording):
    build("detections_2d", entity_path="camera/detections")
    node.publish(detections_2d(box_2d(10, 20, 4, 2)))
    node.publish(detections_2d())
    assert recording.latest("/camera/detections", "LineStrips2D:strips") == []


def test_3d_boxes_are_drawn_in_their_frame(node, build, recording):
    build("detections_3d", entity_path="detections")
    detection = Detection3D(results=[hypothesis("chair", 0.75)], id="3")
    detection.bbox.center.position.x, detection.bbox.center.position.y = 1.0, 2.0
    detection.bbox.center.orientation.z = detection.bbox.center.orientation.w = math.sqrt(0.5)
    detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z = 0.4, 0.6, 1.0
    msg = Detection3DArray(detections=[detection])
    msg.header.frame_id = "map"
    node.publish(msg)

    row = recording.rows("/detections")[0]
    assert row["CoordinateFrame:frame"] == ["map"]
    assert row["Boxes3D:centers"] == [[1.0, 2.0, 0.0]]
    assert row["Boxes3D:half_sizes"] == [pytest.approx([0.2, 0.3, 0.5])]
    assert row["Boxes3D:quaternions"] == [pytest.approx([0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)])]
    assert row["Boxes3D:labels"] == ["chair #3 0.75"]


def test_no_3d_detections_clear_the_boxes(node, build, recording):
    build("detections_3d", entity_path="detections")
    msg = Detection3DArray(detections=[Detection3D()])
    node.publish(msg)
    node.publish(Detection3DArray())
    assert recording.latest("/detections", "Boxes3D:half_sizes") == []


def test_listing_every_class_id_is_an_error(build):
    # Unlisted classes need a class id of their own
    with pytest.raises(ValueError, match="at most 65535 items"):
        build("detections_2d", params={"classes": [{"id": str(i)} for i in range(65536)]})


def test_a_class_listed_twice_is_an_error(build):
    with pytest.raises(ValueError, match="each class id can only be listed once"):
        build("detections_3d", params={"classes": [{"id": "person"}, {"id": "person", "label": "again"}]})
