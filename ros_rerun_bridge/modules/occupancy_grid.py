from __future__ import annotations

from typing import Literal

import numpy as np
from nav_msgs.msg import OccupancyGrid
from rclpy.time import Time

import rerun as rr

from ..base import TopicToComponentModule
from ..registry import REGISTRY
from ..types import StrictParamsModel


class OccupancyGridParams(StrictParamsModel):
    """Parameters for the OccupancyGridModule."""

    colormap: Literal["RvizMap", "RvizCostmap", "Costmap", "Grayscale"] = "RvizMap"
    """Colours as rviz shows maps ("RvizMap") or costmaps ("RvizCostmap")."""
    opacity: float = 1.0
    """Opacity, e.g. below 1 for a costmap drawn over the map."""
    draw_order: float | None = None
    """Grids with a higher draw order are drawn on top, e.g. a costmap over the map."""


@REGISTRY.register("occupancy_grid")
class OccupancyGridModule(TopicToComponentModule):
    """Module for logging nav_msgs/OccupancyGrid maps and costmaps to Rerun as a grid map in the message's frame.

    Module-specific extra configuration parameters:
      - colormap: "RvizMap" | "RvizCostmap" | "Costmap" | "Grayscale" (default "RvizMap")
      - opacity: float (default 1.0)
      - draw_order: float (default: none)
    """

    PARAMS = OccupancyGridParams
    params: OccupancyGridParams  # type hint for self.params

    @classmethod
    def ros_msg_type(cls):  # noqa: D102
        return OccupancyGrid

    def _extract_time(self, msg: OccupancyGrid) -> Time | None:
        return Time.from_msg(msg.header.stamp)

    def handle(self, msg: OccupancyGrid) -> None:  # noqa: D102
        info = msg.info
        # ROS grids start at the bottom-left cell, Rerun's at the top row. Unknown cells (-1) become 255, which the
        # rviz colormaps show as unknown.
        cells = np.flipud(np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)).astype(np.uint8)
        p = info.origin.position
        q = info.origin.orientation
        self.log(
            rr.GridMap(
                data=cells.tobytes(),
                format=rr.components.ImageFormat(
                    width=info.width, height=info.height, color_model="L", channel_datatype="U8"
                ),
                cell_size=info.resolution,
                translation=[p.x, p.y, p.z],
                quaternion=[q.x, q.y, q.z, q.w],
                colormap=getattr(rr.components.Colormap, self.params.colormap),
                opacity=self.params.opacity,
                draw_order=self.params.draw_order,
            ),
            frame_id=msg.header.frame_id,
        )
