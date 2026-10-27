"""Range readings and occupancy grids."""

import math

import numpy as np
import pytest
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Range

import rerun as rr


def reading(value: float) -> Range:
    msg = Range(field_of_view=0.5, min_range=0.02, max_range=2.0, range=value)
    msg.header.frame_id = "inlet_sensor"
    return msg


def test_range_draws_a_cone_along_the_sensor_x_axis(node, build, recording):
    build("range", entity_path="range", params={"colour": [0, 255, 0, 255]})
    node.publish(reading(1.0))

    row = recording.rows("/range")[0]
    rim, *edges = (np.array(strip) for strip in row["LineStrips3D:strips"])
    np.testing.assert_allclose(rim[:, 0], 1.0)
    np.testing.assert_allclose(np.hypot(rim[:, 1], rim[:, 2]), math.tan(0.25), rtol=1e-5)
    assert len(edges) == 4 and all(np.allclose(edge[0], 0.0) for edge in edges)
    assert row["LineStrips3D:colors"] == [0x00FF00FF]


@pytest.mark.parametrize("value", [math.inf, -math.inf, 5.0, 0.0])
def test_range_outside_its_limits_clears_the_cone(node, build, recording, value):
    build("range", entity_path="range")
    node.publish(reading(value))
    assert recording.rows("/range")[0].get("Clear:is_recursive") is not None


def test_fixed_distance_sensor_detections_are_drawn_at_their_range(node, build, recording):
    build("range", entity_path="range")
    msg = Range(field_of_view=0.5, min_range=0.3, max_range=0.3, range=-math.inf)
    node.publish(msg)
    msg.range = math.inf
    node.publish(msg)

    detected, cleared = recording.rows("/range")
    np.testing.assert_allclose(np.array(detected["LineStrips3D:strips"][0])[:, 0], 0.3)
    assert cleared.get("Clear:is_recursive") is not None


def grid() -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.width, msg.info.height, msg.info.resolution = 4, 3, 0.05
    msg.info.origin.position.x, msg.info.origin.position.y = -1.0, -2.0
    msg.info.origin.orientation.w = 1.0
    # Row 0 is the bottom row in ROS
    msg.data = [0, 1, 2, 3, 10, 11, 12, 13, 100, -1, 50, 0]
    return msg


def test_maps_are_flipped_to_top_row_first_with_unknown_cells_kept(node, build, recording):
    build("occupancy_grid", entity_path="map")
    node.publish(grid())

    row = recording.rows("/map")[0]
    cells = np.frombuffer(bytes(row["GridMap:data"][0]), np.uint8).reshape(3, 4)
    np.testing.assert_array_equal(cells, [[100, 255, 50, 0], [10, 11, 12, 13], [0, 1, 2, 3]])
    assert row["GridMap:cell_size"] == pytest.approx([0.05])
    assert row["GridMap:translation"] == [[-1.0, -2.0, 0.0]]
    assert row["GridMap:colormap"] == [rr.components.Colormap.RvizMap.value]
    assert row["CoordinateFrame:frame"] == ["map"]


def test_costmaps_take_their_own_colours_opacity_and_draw_order(node, build, recording):
    build("occupancy_grid", entity_path="costmap", params={"colormap": "RvizCostmap", "opacity": 0.6, "draw_order": 1})
    node.publish(grid())

    row = recording.rows("/costmap")[0]
    assert row["GridMap:colormap"] == [rr.components.Colormap.RvizCostmap.value]
    assert row["GridMap:opacity"] == pytest.approx([0.6])
    assert row["GridMap:draw_order"] == pytest.approx([1.0])
