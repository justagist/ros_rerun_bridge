"""Segmentation masks."""

import numpy as np
import pytest
from sensor_msgs.msg import Image


def mask(pixels: np.ndarray, encoding: str) -> Image:
    msg = Image(height=pixels.shape[0], width=pixels.shape[1], encoding=encoding, step=pixels.strides[0])
    msg.data = pixels.tobytes()
    msg.header.frame_id = "camera"
    return msg


@pytest.mark.parametrize(
    ("encoding", "pixels"),
    [
        ("mono8", np.array([[0, 1], [2, 255]], np.uint8)),
        ("8UC1", np.array([[0, 1], [2, 255]], np.uint8)),
        ("mono16", np.array([[0, 1], [300, 65535]], np.uint16)),
        ("16UC1", np.array([[0, 1], [300, 65535]], np.uint16)),
    ],
)
def test_masks_keep_their_class_ids_on_the_image_plane(node, build, recording, encoding, pixels):
    build("segmentation_image", entity_path="camera/segmentation")
    node.publish(mask(pixels, encoding))

    row = recording.rows("/camera/segmentation")[0]
    ids = np.frombuffer(bytes(row["SegmentationImage:buffer"][0]), dtype=pixels.dtype).reshape(pixels.shape)
    np.testing.assert_array_equal(ids, pixels)
    assert row["CoordinateFrame:frame"] == ["camera_image_plane"]


def test_classes_are_logged_with_their_labels_and_colours(build, recording):
    classes = [{"id": 0, "colour": [0, 0, 0, 0]}, {"id": 1, "label": "person", "colour": [220, 40, 40, 255]}]
    build("segmentation_image", entity_path="camera/segmentation", params={"classes": classes})

    # One context, holding every class
    [classes] = recording.latest("/camera/segmentation", "AnnotationContext:context")
    assert [c["class_description"]["info"] for c in classes] == [
        {"id": 0, "label": None, "color": 0x00000000},
        {"id": 1, "label": "person", "color": 0xDC2828FF},
    ]


def test_no_classes_log_no_annotation_context(build, recording):
    build("segmentation_image", entity_path="camera/segmentation")
    assert "/camera/segmentation" not in recording.entities()


def test_images_that_are_not_class_ids_are_an_error(node, build):
    build("segmentation_image", entity_path="camera/segmentation")
    node.publish(mask(np.zeros((2, 2), np.float32), "32FC1"))
    assert "'32FC1' is not a class id encoding" in node.log_messages[-2][1]


def test_a_class_listed_twice_is_an_error(build):
    with pytest.raises(ValueError, match="each class id can only be listed once"):
        build("segmentation_image", params={"classes": [{"id": 1}, {"id": 1, "label": "again"}]})
