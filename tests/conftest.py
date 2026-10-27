"""Shared fixtures: a fake ROS node, so modules run without a ROS graph, and an in-memory Rerun recording."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import pyarrow as pa
import pytest
from rclpy.time import Time

import rerun as rr
import rerun.chunk

from ros_rerun_bridge.base import BridgeConfig
from ros_rerun_bridge.builder import BridgeBuilder
from ros_rerun_bridge.context import BridgeContext


@dataclass
class FakeSubscription:
    msg_type: type
    topic: str
    callback: Callable[[Any], None]
    qos: Any


class FakeLogger:
    """Collects log calls as (level, message) pairs."""

    def __init__(self, messages: list[tuple[str, str]]) -> None:
        self._messages = messages

    def __getattr__(self, level: str) -> Callable[..., None]:
        return lambda message, **_: self._messages.append((level, message))


class FakeClock:
    def __init__(self, node: FakeNode) -> None:
        self._node = node

    def now(self) -> Time:
        return Time(nanoseconds=self._node.now_ns)


@dataclass
class FakeNode:
    """Stands in for rclpy.node.Node: records subscriptions and timers, and has a clock the test sets."""

    now_ns: int = 1_000_000_000_000
    subscriptions: list[FakeSubscription] = field(default_factory=list)
    timers: list[Callable[[], None]] = field(default_factory=list)
    log_messages: list[tuple[str, str]] = field(default_factory=list)

    def create_subscription(self, msg_type, topic, callback, qos, callback_group=None):
        self.subscriptions.append(FakeSubscription(msg_type, topic, callback, qos))
        return self.subscriptions[-1]

    def create_timer(self, period_s, callback, callback_group=None):
        self.timers.append(callback)

    def get_logger(self) -> FakeLogger:
        return FakeLogger(self.log_messages)

    def get_clock(self) -> FakeClock:
        return FakeClock(self)

    def publish(self, msg: Any, index: int = -1) -> None:
        """Deliver a message to a module's subscription, as the executor would."""
        self.subscriptions[index].callback(msg)


@pytest.fixture
def node() -> FakeNode:
    return FakeNode()


@pytest.fixture
def context() -> BridgeContext:
    return BridgeContext()


@pytest.fixture
def build(node: FakeNode, context: BridgeContext) -> Callable[..., Any]:
    """Build one module through the real builder from a config entry, e.g. build("tf", params={"static": True})."""

    def _build(module: str, *, entity_path: str = "test", topic: str = "/test", **spec: Any) -> Any:
        config = BridgeConfig(
            modules=[{"name": module, "module": module, "topic": topic, "entity_path": entity_path, **spec}]
        )
        return BridgeBuilder(node).build_modules(config, context)[0]

    return _build


class Recording:
    """An in-memory Rerun recording, read back as rows per entity.

    Read it once the test has logged everything: later data cannot be read, as only the first read of the memory
    recording carries the recording's identity.
    """

    def __init__(self, path: Path) -> None:
        self._stream = rr.RecordingStream("ros_rerun_bridge_test")
        self._memory = self._stream.memory_recording()
        self._path = path
        self._chunks: list[rerun.chunk.Chunk] | None = None

    def chunks(self) -> list[rerun.chunk.Chunk]:
        """All chunks logged up to the first read."""
        if self._chunks is None:
            self._stream.flush()
            self._path.write_bytes(self._memory.drain_as_bytes())
            self._chunks = rerun.chunk.RrdReader(self._path).stream().to_chunks()
        return self._chunks

    def close(self) -> None:
        """Fail if data was logged after the recording was read, which the test would not have seen."""
        self._stream.flush()
        if self._chunks is None:
            # Unread data would otherwise be reported as dropped when the recording is freed
            self._memory.drain_as_bytes()
        else:
            assert self._memory.num_msgs() == 0, "data was logged after the recording was read"

    def entities(self) -> set[str]:
        return {chunk.entity_path for chunk in self.chunks()}

    def rows(self, entity_path: str) -> list[dict[str, Any]]:
        """Rows logged on an entity, static ones first, then in time order, then in logging order.

        Each row maps "Archetype:component" to its value, plus "static" and, for temporal rows, "ros_time" in ns.
        """
        rows = []
        for chunk in self.chunks():
            if chunk.entity_path != entity_path:
                continue
            batch = chunk.to_record_batch()
            for i in range(batch.num_rows):
                # Row ids are time-ordered: they order rows logged at the same ROS time
                row: dict[str, Any] = {"static": chunk.is_static, "row_id": batch.column(0)[i].as_py()}
                for name in batch.schema.names:
                    column = batch.column(name)
                    if name == "ros_time":
                        row[name] = column.cast(pa.int64())[i].as_py()
                    elif ":" in name:
                        row[name] = column[i].as_py()
                rows.append(row)
        return sorted(rows, key=lambda row: (not row["static"], row.get("ros_time") or 0, row["row_id"]))

    def latest(self, entity_path: str, component: str) -> Any:
        """The last value logged for a component on an entity, or None."""
        values = [row[component] for row in self.rows(entity_path) if row.get(component) is not None]
        return values[-1] if values else None


@pytest.fixture
def recording(tmp_path: Path) -> Recording:
    """Make an in-memory recording the global one, in strict mode as the bridge runs, for the test's duration."""
    recording = Recording(tmp_path / "test.rrd")
    previous = rr.set_global_data_recording(recording._stream)
    rr.set_strict_mode(True)
    yield recording
    rr.set_strict_mode(False)
    rr.set_global_data_recording(previous)
    recording.close()
