"""Simulator-independent scenario data models."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class ScenarioObject:
    """Static object placed in a scenario.

    The fields intentionally mirror common ADAS scenario concepts: object type,
    pose, dimensions, and optional metadata. Exporters can translate this shape
    into OpenSCENARIO, simulator JSON, or other interchange formats.
    """

    object_id: str
    object_type: str
    x: float
    y: float
    heading: float
    length: float
    width: float
    slot_index: int | None = None

    def get_corners(self, margin: float = 0.0) -> np.ndarray:
        """Return object footprint corners in global coordinates with optional inflation."""
        if not np.isfinite(margin) or margin < 0.0:
            raise ValueError("footprint margin must be finite and nonnegative")
        cos_h = np.cos(self.heading)
        sin_h = np.sin(self.heading)
        half_length = self.length / 2.0 + margin
        half_width = self.width / 2.0 + margin
        corners_local = np.array(
            [
                [-half_length, -half_width],
                [-half_length, half_width],
                [half_length, half_width],
                [half_length, -half_width],
            ]
        )

        corners_global = np.zeros_like(corners_local)
        for i, corner in enumerate(corners_local):
            corners_global[i, 0] = self.x + corner[0] * cos_h - corner[1] * sin_h
            corners_global[i, 1] = self.y + corner[0] * sin_h + corner[1] * cos_h
        return corners_global

    def as_dict(self) -> dict[str, Any]:
        """Return an export-friendly dictionary representation."""
        return asdict(self)
