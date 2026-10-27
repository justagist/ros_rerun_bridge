import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

import rerun as rr

from ros_rerun_bridge.modules.compressed_image import media_type_from_format
from ros_rerun_bridge.modules.image import image_from_msg


def ros_image(pixels: np.ndarray, encoding: str, padding: int = 0, big_endian: bool = False) -> Image:
    """Build a ROS image from an array, with `padding` bytes at the end of each row."""
    data = pixels.astype(pixels.dtype.newbyteorder(">" if big_endian else "<"))
    rows = data.reshape(pixels.shape[0], -1).view(np.uint8)
    rows = np.hstack([rows, np.full((rows.shape[0], padding), 0xEE, np.uint8)])
    return Image(
        height=pixels.shape[0],
        width=pixels.shape[1],
        encoding=encoding,
        is_bigendian=int(big_endian),
        step=rows.shape[1],
        data=rows.tobytes(),
    )


RNG = np.random.default_rng(0)
ENCODINGS = [
    ("rgb8", RNG.integers(0, 255, (3, 5, 3), dtype=np.uint8), "RGB"),
    ("bgr8", RNG.integers(0, 255, (3, 5, 3), dtype=np.uint8), "BGR"),
    ("bgra8", RNG.integers(0, 255, (3, 5, 4), dtype=np.uint8), "BGRA"),
    ("rgba8", RNG.integers(0, 255, (3, 5, 4), dtype=np.uint8), "RGBA"),
    ("8UC1", RNG.integers(0, 255, (3, 5), dtype=np.uint8), "L"),
    ("16UC1", RNG.integers(0, 65535, (3, 5), dtype=np.uint16), "L"),
    ("mono8", RNG.integers(0, 255, (3, 5), dtype=np.uint8), "L"),
    ("mono16", RNG.integers(0, 65535, (3, 5), dtype=np.uint16), "L"),
    ("rgb16", RNG.integers(0, 65535, (3, 5, 3), dtype=np.uint16), "RGB"),
    ("32FC1", RNG.random((3, 5), dtype=np.float32), "L"),
]


@pytest.mark.parametrize(("encoding", "pixels", "colour_model"), ENCODINGS, ids=[e[0] for e in ENCODINGS])
@pytest.mark.parametrize(("padding", "big_endian"), [(0, False), (3, False), (2, True)], ids=["plain", "padded", "be"])
def test_images_keep_their_exact_pixels_and_colour_model(encoding, pixels, colour_model, padding, big_endian):
    image = image_from_msg(ros_image(pixels, encoding, padding, big_endian))
    image_format = image.format.as_arrow_array().to_pylist()[0]
    buffer = np.frombuffer(bytes(image.buffer.as_arrow_array().to_pylist()[0]), dtype=pixels.dtype)
    assert (image_format["width"], image_format["height"]) == (5, 3)
    assert image_format["color_model"] == rr.ColorModel[colour_model].value
    np.testing.assert_array_equal(buffer.reshape(pixels.shape), pixels)


@pytest.mark.parametrize(
    ("encoding", "pixel_format", "step"),
    [("bayer_rggb8", rr.PixelFormat.BayerRGGB8, 6), ("yuv422_yuy2", rr.PixelFormat.YUY2, 12)],
)
def test_packed_encodings_are_passed_to_rerun_to_decode(encoding, pixel_format, step):
    raw = RNG.integers(0, 255, (4, step), dtype=np.uint8)
    image = image_from_msg(Image(height=4, width=6, encoding=encoding, step=step, data=raw.tobytes()))
    assert image.format.as_arrow_array().to_pylist()[0]["pixel_format"] == pixel_format.value
    assert bytes(image.buffer.as_arrow_array().to_pylist()[0]) == raw.tobytes()


def test_unsupported_encodings_are_an_error():
    with pytest.raises(ValueError, match="nv21"):
        image_from_msg(Image(height=1, width=2, encoding="nv21", step=2, data=bytes(2)))


def test_images_are_placed_on_the_camera_image_plane(node, build, recording):
    build("image", entity_path="camera/image")
    msg = ros_image(np.zeros((2, 2), np.uint8), "mono8")
    msg.header.frame_id = "camera"
    node.publish(msg)
    assert recording.rows("/camera/image")[0]["CoordinateFrame:frame"] == ["camera_image_plane"]


@pytest.mark.parametrize(
    ("image_format", "media_type"),
    [("jpeg", "image/jpeg"), ("bgr8; jpeg compressed bgr8", "image/jpeg"), ("png", "image/png")],
)
def test_compressed_formats_map_to_media_types(image_format, media_type):
    assert media_type_from_format(image_format) == media_type


def test_compressed_depth_is_not_supported():
    with pytest.raises(ValueError, match="compressedDepth"):
        media_type_from_format("16UC1; compressedDepth png")


def test_compressed_images_are_logged_as_is_on_the_image_plane(node, build, recording):
    build("compressed_image", entity_path="camera/compressed")
    jpeg = bytes([0xFF, 0xD8, 0xFF, 0xE0]) + bytes(20)
    msg = CompressedImage(format="jpeg", data=jpeg)
    msg.header.frame_id = "camera"
    node.publish(msg)

    row = recording.rows("/camera/compressed")[0]
    assert bytes(row["EncodedImage:blob"][0]) == jpeg
    assert row["EncodedImage:media_type"] == ["image/jpeg"]
    assert row["CoordinateFrame:frame"] == ["camera_image_plane"]


def test_camera_info_links_the_camera_frame_to_its_image_plane(node, build, recording):
    build("camera_info", entity_path="camera/info")
    msg = CameraInfo(width=640, height=480, k=[500.0, 0.0, 320.0, 0.0, 510.0, 240.0, 0.0, 0.0, 1.0])
    msg.header.frame_id = "camera"
    node.publish(msg)

    row = recording.rows("/camera/info")[0]
    assert row["Pinhole:parent_frame"] == ["camera"]
    assert row["Pinhole:child_frame"] == ["camera_image_plane"]
    assert row["Pinhole:resolution"] == [[640.0, 480.0]]
    # Rerun stores the matrix column-major
    np.testing.assert_allclose(
        np.reshape(row["Pinhole:image_from_camera"][0], (3, 3), order="F"), np.reshape(msg.k, (3, 3))
    )
