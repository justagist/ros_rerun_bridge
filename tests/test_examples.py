"""The example configs build: every module, its params and its fields are valid. Their layouts match layouts.py."""

import importlib.util
import re
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

# Blueprint ids, as text, and the viewport's root container id as its 16 bytes: new ones each time a layout is saved
_IDS = re.compile(r"[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}|\[(?:\d+, ){15}\d+\]")


@pytest.mark.parametrize("config", CONFIGS, ids=[config.parent.name for config in CONFIGS])
# `recording`, as some modules log their styles when built
def test_example_config_builds(config, node, context, recording):
    cfg = load_config(config)
    assert len(BridgeBuilder(node).build_modules(cfg, context)) == len(cfg.modules)


def test_layouts_py_has_a_layout_for_every_example_with_one():
    with_layout = {config.parent.name for config in CONFIGS if load_config(config).blueprint}
    assert set(layouts.LAYOUTS) == with_layout


def layout_rows(path: Path) -> list[str]:
    """A saved layout's rows, with their ids replaced, in a fixed order."""
    reader = rerun.chunk.RrdReader(path)
    rows = []
    for chunk in reader.stream(store=reader.blueprints()[0]).to_chunks():
        batch = chunk.to_record_batch()
        data = sorted((name, batch.column(name).to_pylist()) for name in batch.schema.names if ":" in name)
        rows.append(_IDS.sub("ID", f"{chunk.entity_path} {data}"))
    return sorted(rows)


@pytest.mark.parametrize("name", sorted(layouts.LAYOUTS))
def test_saved_layout_matches_layouts_py(name, tmp_path):
    # Fails when layouts.py is changed without running it
    saved = tmp_path / "layout.rbl"
    rrb.Blueprint(layouts.LAYOUTS[name]).save("ros_rerun_bridge", saved)
    assert layout_rows(saved) == layout_rows(EXAMPLES / name / "layout.rbl")
