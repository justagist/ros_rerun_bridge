"""The example configs build: every module, its params and its fields are valid. Their layouts match layouts.py."""

import importlib.util
import re
import uuid
from pathlib import Path

import pytest

import rerun.blueprint as rrb
import rerun.chunk

from ros_rerun_bridge.builder import BridgeBuilder
from ros_rerun_bridge.main import load_config

EXAMPLES = Path(__file__).parents[1] / "examples"
CONFIGS = sorted(EXAMPLES.glob("*/config.yaml"))

_spec = importlib.util.spec_from_file_location("layouts", EXAMPLES / "layouts.py")
layouts = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(layouts)

_UUID = re.compile(r"[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}")


@pytest.mark.parametrize("config", CONFIGS, ids=[config.parent.name for config in CONFIGS])
# `recording`, as some modules log their styles when built
def test_example_config_builds(config, node, context, recording):
    cfg = load_config(config)
    assert len(BridgeBuilder(node).build_modules(cfg, context)) == len(cfg.modules)


def test_layouts_py_has_a_layout_for_every_example_with_one():
    with_layout = {config.parent.name for config in CONFIGS if load_config(config).blueprint}
    assert set(layouts.LAYOUTS) == with_layout


def layout_rows(path: Path) -> list[str]:
    """A saved layout's rows, in a fixed order, with its ids, new each time it is saved, named by where they are."""
    reader = rerun.chunk.RrdReader(path)
    rows: dict[str, dict] = {}
    for chunk in reader.stream(store=reader.blueprints()[0]).to_chunks():
        batch = chunk.to_record_batch()
        row = rows.setdefault(chunk.entity_path, {})
        row.update({name: batch.column(name).to_pylist() for name in batch.schema.names if ":" in name})
    # Walk the layout from its root container, so the ids are named in the order of each container's children
    root = uuid.UUID(bytes=bytes(rows["/viewport"]["ViewportBlueprint:root_container"][0][0]))
    names: dict[str, str] = {}
    pending = [f"container/{root}"]
    while pending:
        child = pending.pop(0)
        names[child.split("/")[1]] = f"id{len(names)}"
        pending += rows.get(f"/{child}", {}).get("ContainerBlueprint:contents", [[]])[0]
    rows["/viewport"]["ViewportBlueprint:root_container"] = [str(root)]
    text = [f"{entity} {sorted(row.items())}" for entity, row in rows.items()]
    return sorted(_UUID.sub(lambda match: names[match.group()], line) for line in text)


@pytest.mark.parametrize("name", sorted(layouts.LAYOUTS))
def test_saved_layout_matches_layouts_py(name, tmp_path):
    # Fails when layouts.py is changed without running it
    saved = tmp_path / "layout.rbl"
    rrb.Blueprint(layouts.LAYOUTS[name]).save("ros_rerun_bridge", saved)
    assert layout_rows(saved) == layout_rows(EXAMPLES / name / "layout.rbl")


def test_layouts_with_their_views_swapped_differ(tmp_path):
    views = [rrb.Spatial2DView(origin="/camera/image"), rrb.Spatial2DView(origin="/camera/depth")]
    rrb.Blueprint(rrb.Vertical(*views)).save("ros_rerun_bridge", tmp_path / "a.rbl")
    rrb.Blueprint(rrb.Vertical(*reversed(views))).save("ros_rerun_bridge", tmp_path / "b.rbl")
    assert layout_rows(tmp_path / "a.rbl") != layout_rows(tmp_path / "b.rbl")
