"""Depth images, raw and compressed."""

import struct

import numpy as np
import pytest
from sensor_msgs.msg import CompressedImage, Image

import rerun as rr

from ros_rerun_bridge.modules.compressed_depth_image import encoded_depth_from_msg

PNG = b"\x89PNG\r\n\x1a\n" + bytes(16)
# compressed_depth_image_transport's config header: the compression format and two quantisation parameters
HEADER = struct.pack("<iff", 0, 0.0, 0.0)


def depth_image(pixels: np.ndarray, encoding: str) -> Image:
    msg = Image(height=pixels.shape[0], width=pixels.shape[1], encoding=encoding, step=pixels.strides[0])
    msg.data = pixels.tobytes()
    msg.header.frame_id = "camera"
    return msg


@pytest.mark.parametrize(
    ("encoding", "pixels", "meter"),
    [
        ("16UC1", np.array([[500, 1000], [1500, 0]], np.uint16), 1000.0),
        ("32FC1", np.array([[0.5, 1.0], [1.5, np.nan]], np.float32), 1.0),
    ],
)
def test_depth_images_keep_their_depths_and_units_on_the_image_plane(node, build, recording, encoding, pixels, meter):
    build("depth_image", entity_path="camera/depth")
    node.publish(depth_image(pixels, encoding))

    row = recording.rows("/camera/depth")[0]
    depths = np.frombuffer(bytes(row["DepthImage:buffer"][0]), dtype=pixels.dtype).reshape(pixels.shape)
    np.testing.assert_array_equal(depths, pixels)
    assert row["DepthImage:meter"] == [meter]
    assert row["CoordinateFrame:frame"] == ["camera_image_plane"]


def test_depth_display_params_are_passed_to_rerun(node, build, recording):
    params = {"colormap": "Viridis", "depth_range": [200, 4000], "point_fill_ratio": 0.5}
    build("depth_image", entity_path="camera/depth", params=params)
    node.publish(depth_image(np.zeros((2, 2), np.uint16), "16UC1"))

    row = recording.rows("/camera/depth")[0]
    assert row["DepthImage:colormap"] == [rr.components.Colormap.Viridis.value]
    assert row["DepthImage:depth_range"] == [[200.0, 4000.0]]
    assert row["DepthImage:point_fill_ratio"] == pytest.approx([0.5])


def test_images_that_are_not_depth_are_an_error(node, build, recording):
    build("depth_image", entity_path="camera/depth")
    node.publish(depth_image(np.zeros((2, 2), np.uint8), "mono8"))
    assert "'mono8' is not a depth encoding" in node.log_messages[-2][1]


def compressed(image_format: str, data: bytes) -> CompressedImage:
    return CompressedImage(format=image_format, data=data)


def test_png_depth_has_its_ros_header_removed():
    data, media_type = encoded_depth_from_msg(compressed("16UC1; compressedDepth png", HEADER + PNG))
    assert (data, media_type) == (PNG, "image/png")


def test_rvl_depth_is_passed_on_with_its_header_for_rerun_to_read():
    # Rerun reads ROS's RVL header itself, including 32FC1's quantisation
    rvl = HEADER + struct.pack("<II", 2, 2) + bytes(8)
    data, media_type = encoded_depth_from_msg(compressed("32FC1; compressedDepth rvl", rvl))
    assert (data, media_type) == (rvl, "application/rvl")


@pytest.mark.parametrize(
    ("image_format", "data", "error"),
    [
        # Quantised inverse depth: Rerun would show wrong distances
        ("32FC1; compressedDepth png", HEADER + PNG, "only 16UC1 depth can be sent as PNG"),
        ("16UC1; compressedDepth png", PNG, "not a PNG after its header"),
        ("8UC1; compressedDepth rvl", HEADER + PNG, "RVL depth must be 16UC1 or 32FC1"),
        ("16UC1; compressedDepth tiff", HEADER + PNG, "unsupported compressed depth format"),
        ("png", PNG, "unsupported compressed depth format"),
    ],
)
def test_unsupported_compressed_depth_is_an_error(image_format, data, error):
    with pytest.raises(ValueError, match=error):
        encoded_depth_from_msg(compressed(image_format, data))


def test_compressed_depth_images_are_logged_on_the_image_plane(node, build, recording):
    build("compressed_depth_image", entity_path="camera/depth", params={"colormap": "Turbo"})
    msg = compressed("16UC1; compressedDepth png", HEADER + PNG)
    msg.header.frame_id = "camera"
    node.publish(msg)

    row = recording.rows("/camera/depth")[0]
    assert bytes(row["EncodedDepthImage:blob"][0]) == PNG
    assert row["EncodedDepthImage:media_type"] == ["image/png"]
    assert row["EncodedDepthImage:colormap"] == [rr.components.Colormap.Turbo.value]
    assert row["CoordinateFrame:frame"] == ["camera_image_plane"]
