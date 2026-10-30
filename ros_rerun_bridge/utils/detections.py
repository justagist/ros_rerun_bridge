"""Classes, labels and colours of vision_msgs detections, for the modules that draw them: `detections_2d` and
`detections_3d`."""

from __future__ import annotations

import zlib
from collections.abc import Sequence
from typing import Any

from pydantic import model_validator

import rerun as rr

from ..types import Rgba, StrictParamsModel

# Rerun's class ids are 16-bit
_CLASS_IDS = 65536


class DetectionClass(StrictParamsModel):
    """How one class is shown."""

    id: str
    """The class's `class_id` in the detections' hypotheses, e.g. `person` or `17`."""
    label: str | None = None
    """Name shown for the class in its detections' labels. Its id if unset."""
    colour: Rgba | None = None
    """RGBA colour of the class's detections, as uint8. One Rerun picks for the class if unset."""


class DetectionParams(StrictParamsModel):
    """Parameters for the modules that draw detections."""

    classes: list[DetectionClass] = []
    """Label and colour per class. Classes not listed are shown with their id, in a colour Rerun picks for each."""

    @model_validator(mode="after")
    def _check_ids(self) -> DetectionParams:
        """A class listed twice would leave which of its labels and colours is shown unclear."""
        ids = [c.id for c in self.classes]
        if len(set(ids)) != len(ids):
            raise ValueError("each class id can only be listed once")
        return self


class DetectionClasses:
    """Gives each detection a Rerun class id, for its colour, and a label: its best hypothesis's class, its tracking id
    if it has one, and its best hypothesis's score, e.g. `person #7 0.92`.

    Listed classes have the class ids 0 to n-1, in an annotation context holding their colours. Other classes have an
    id above those, from a checksum of their `class_id`, so each keeps the colour Rerun picks for it across messages
    and runs.
    """

    def __init__(self, entity_path: str, classes: list[DetectionClass]) -> None:
        """Log the listed classes' colours.

        Args:
            entity_path: The module's entity path, which its detections are logged on.
            classes: The classes listed in the module's params.
        """
        self._listed = {c.id: (i, c.label or c.id) for i, c in enumerate(classes)}
        if classes:
            context = rr.AnnotationContext([rr.AnnotationInfo(id=i, color=c.colour) for i, c in enumerate(classes)])
            rr.log(entity_path, context, static=True)

    def class_ids_and_labels(self, detections: Sequence[Any]) -> tuple[list[int], list[str]]:
        """Each detection's Rerun class id and label.

        Args:
            detections: vision_msgs Detection2D or Detection3D messages.

        Returns:
            The class ids and labels, in the detections' order.
        """
        class_ids, labels = [], []
        for detection in detections:
            best = max(detection.results, key=lambda result: result.hypothesis.score, default=None)
            class_id, name = self._class(best.hypothesis.class_id if best else "")
            parts = [name] if best else []
            if detection.id:
                parts.append(f"#{detection.id}")
            if best:
                parts.append(f"{best.hypothesis.score:.2f}")
            class_ids.append(class_id)
            labels.append(" ".join(parts))
        return class_ids, labels

    def _class(self, class_id: str) -> tuple[int, str]:
        """A class's Rerun class id and the name its detections are labelled with.

        Args:
            class_id: The class's `class_id` in a hypothesis.

        Returns:
            Its Rerun class id and name.
        """
        if class_id in self._listed:
            return self._listed[class_id]
        n_listed = len(self._listed)
        return n_listed + zlib.crc32(class_id.encode()) % (_CLASS_IDS - n_listed), class_id
