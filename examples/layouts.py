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


def plots(origin: str) -> rrb.TimeSeriesView:
    """A time series view of the entities under `origin`."""
    return rrb.TimeSeriesView(origin=origin, time_ranges=RECENT)


LAYOUTS = {
    "robot_arm": rrb.Horizontal(scene([0.9, -0.9, 0.75], [0.1, 0.0, 0.4]), plots("/plots"), column_shares=[2, 1]),
    "mobile_robot": rrb.Horizontal(
        scene([-1.0, -6.5, 4.5], [-1.0, 0.0, 0.0]), plots("/plots/odom"), column_shares=[2, 1]
    ),
    "camera": rrb.Horizontal(scene([-0.5, -0.8, 1.35], [0.7, 0.0, 1.0]), rrb.Spatial2DView(origin="/camera/image")),
    "point_cloud": scene([2.4, -2.4, 2.2], [0.0, 0.0, 1.0]),
    "range_sensor": rrb.Horizontal(scene([0.9, -1.6, 1.0], [0.6, 0.0, 0.25]), plots("/plots/range")),
    "markers": scene([1.0, -2.6, 1.5], [1.0, 0.0, 0.2]),
    "series_styles": rrb.Vertical(plots("/plots/default"), plots("/plots/points"), plots("/plots/mode")),
}

if __name__ == "__main__":
    for name, layout in LAYOUTS.items():
        # Saved for the bridge's application id, so it applies to the recordings the bridge makes
        rrb.Blueprint(layout).save("ros_rerun_bridge", EXAMPLES / name / "layout.rbl")
        print(f"wrote {name}/layout.rbl")
