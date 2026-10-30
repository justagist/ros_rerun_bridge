"""Write each example's viewer layout to `<example>/layout.rbl`, which its config opens the viewer with.

After changing a layout here, run from the repo root: uv run python examples/layouts.py
"""

from pathlib import Path

import rerun.blueprint as rrb

EXAMPLES = Path(__file__).parent

# Plots scroll with the time cursor, showing the last 6 s
RECENT = rrb.VisibleTimeRange(
    "ros_time", start=rrb.TimeRangeBoundary.cursor_relative(seconds=-6.0), end=rrb.TimeRangeBoundary.cursor_relative()
)


def scene(position: list[float], target: list[float]) -> rrb.Spatial3DView:
    """A 3D view of the whole scene, with the camera at `position` looking at `target`."""
    camera = rrb.EyeControls3D(kind=rrb.Eye3DKind.Orbital, position=position, look_target=target)
    return rrb.Spatial3DView(origin="/", eye_controls=camera)


def plots(origin: str, y_range: tuple[float, float]) -> rrb.TimeSeriesView:
    """A time series view of the entities under `origin`, with its y axis fixed to `y_range`."""
    # Fixed, rather than fitted to the values shown, so the axis does not jump as the plot scrolls
    y_axis = rrb.ScalarAxis(range=y_range, zoom_lock=True)
    return rrb.TimeSeriesView(origin=origin, time_ranges=RECENT, axis_y=y_axis)


LAYOUTS = {
    "robot_arm": rrb.Horizontal(
        scene([0.9, -0.9, 0.75], [0.1, 0.0, 0.4]),
        rrb.Vertical(plots("/plots", (-1.6, 1.6)), rrb.BarChartView(origin="/bars/joints")),
        column_shares=[2, 1],
    ),
    "mobile_robot": rrb.Horizontal(
        scene([-1.0, -6.5, 4.5], [-1.0, 0.0, 0.0]), plots("/plots/odom", (0.0, 0.6)), column_shares=[2, 1]
    ),
    "camera": rrb.Horizontal(
        scene([-1.2, -2.2, 2.2], [1.5, 0.0, 1.0]),
        rrb.Vertical(rrb.Spatial2DView(origin="/camera/image"), rrb.Spatial2DView(origin="/camera/depth")),
        column_shares=[2, 1],
    ),
    "perception": rrb.Horizontal(
        scene([-0.9, -1.8, 1.7], [0.8, 0.0, 0.95]),
        rrb.Vertical(
            rrb.Spatial2DView(origin="/camera/image", name="Image, mask and detections"),
            rrb.Spatial2DView(origin="/camera/image/segmentation", name="Mask"),
        ),
        column_shares=[3, 2],
    ),
    "point_cloud": scene([2.4, -2.4, 2.2], [0.0, 0.0, 1.0]),
    "range_sensor": rrb.Horizontal(scene([0.9, -1.6, 1.0], [0.6, 0.0, 0.25]), plots("/plots/range", (0.0, 2.0))),
    "markers": scene([1.0, -2.6, 1.5], [1.0, 0.0, 0.2]),
    "states": rrb.StateTimelineView(origin="/states", time_ranges=RECENT),
    "logs_and_diagnostics": rrb.Horizontal(
        rrb.TextLogView(origin="/logs", name="Logs"),
        rrb.Vertical(
            rrb.StateTimelineView(origin="/diagnostics", time_ranges=RECENT),
            rrb.TextLogView(origin="/diagnostics", name="Diagnostic messages"),
        ),
    ),
    "geometry": scene([0.6, -2.6, 1.8], [0.8, 0.0, 0.4]),
    "series_styles": rrb.Vertical(
        plots("/plots/default", (-1.1, 1.1)), plots("/plots/points", (-1.1, 1.1)), plots("/plots/mode", (-0.2, 2.2))
    ),
}

if __name__ == "__main__":
    for name, layout in LAYOUTS.items():
        # Saved for the bridge's application id, so it applies to the recordings the bridge makes
        rrb.Blueprint(layout).save("ros_rerun_bridge", EXAMPLES / name / "layout.rbl")
        print(f"wrote {name}/layout.rbl")
