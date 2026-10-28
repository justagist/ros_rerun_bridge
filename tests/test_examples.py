"""The example configs build: every module, its params and its fields are valid."""

from pathlib import Path

import pytest

from ros_rerun_bridge.builder import BridgeBuilder
from ros_rerun_bridge.main import load_config

CONFIGS = sorted((Path(__file__).parents[1] / "examples").glob("*/config.yaml"))


@pytest.mark.parametrize("config", CONFIGS, ids=[config.parent.name for config in CONFIGS])
def test_example_config_builds(config, node, context, recording):
    cfg = load_config(config)
    assert len(BridgeBuilder(node).build_modules(cfg, context)) == len(cfg.modules)
