"""The example configs build: every module, its params and its fields are valid."""

from pathlib import Path

import pytest

from ros_rerun_bridge.builder import BridgeBuilder
from ros_rerun_bridge.config import load_config

EXAMPLES = Path(__file__).parents[1] / "examples"
CONFIGS = sorted(EXAMPLES.glob("*/config.yaml"))


@pytest.mark.parametrize("config", CONFIGS, ids=[config.parent.name for config in CONFIGS])
# `recording`, as some modules log their styles when built
def test_example_config_builds(config, node, context, recording):
    BridgeBuilder(node).build_modules(load_config(config), context)
