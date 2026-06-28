"""Scenario artifact read/write helpers."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .parking import ParkingScenario


def save_scenario(scenario: ParkingScenario, path: Path) -> Path:
    """Write a scenario artifact as JSON."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(scenario.as_dict(), f, indent=2, sort_keys=True)
        f.write("\n")
    return path


def load_parking_scenario(path: Path) -> ParkingScenario:
    """Load a parking scenario artifact from JSON."""
    with path.open("r", encoding="utf-8") as f:
        data: dict[str, Any] = json.load(f)
    return ParkingScenario.from_dict(data)
