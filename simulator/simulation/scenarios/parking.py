"""Parking scenario models and factories."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from simulator.simulation.parking_lot import ParkingLot, ParkingSlot

from .models import ScenarioObject


@dataclass(frozen=True)
class ParkingScenario:
    """Immutable parking scenario consumed by the simulator."""

    scenario_id: str
    lot: ParkingLot
    goal_slot_index: int
    parked_cars: tuple[ScenarioObject, ...]

    @property
    def goal_slot(self) -> ParkingSlot:
        return self.lot.get_slot(self.goal_slot_index)

    def get_parked_cars(self) -> list[ScenarioObject]:
        """Return parked cars currently placed in the scenario."""
        return list(self.parked_cars)

    def get_random_empty_slot(self, rng: np.random.Generator | None = None) -> tuple[int, ParkingSlot | None]:
        """Return a random slot that is not occupied by a parked car."""
        occupied_indices = {car.slot_index for car in self.parked_cars if car.slot_index is not None}
        empty_indices = [
            index for index in range(self.lot.num_slots)
            if index not in occupied_indices
        ]
        if not empty_indices:
            return -1, None

        generator = rng or np.random.default_rng()
        index = int(generator.choice(empty_indices))
        return index, self.lot.get_slot(index)

    def as_dict(self) -> dict[str, Any]:
        """Return an export-friendly scenario representation."""
        return {
            "scenario_id": self.scenario_id,
            "type": "parking",
            "goal_slot_index": self.goal_slot_index,
            "parking_lot": {
                "rows": self.lot.rows,
                "cols": self.lot.cols,
                "layout": self.lot.layout,
                "origin_x": self.lot.origin_x,
                "origin_y": self.lot.origin_y,
                "slot_width": self.lot.slot_width,
                "slot_length": self.lot.slot_length,
                "gap_between_slots": self.lot.gap_between_slots,
                "aisle_width": self.lot.aisle_width,
                "slot_orientation_deg": float(np.rad2deg(self.lot.slot_orientation_rad)),
                "slot_orientation_rad": self.lot.slot_orientation_rad,
                "lot_width": self.lot.lot_width,
                "lot_length": self.lot.lot_length,
            },
            "objects": [car.as_dict() for car in self.parked_cars],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ParkingScenario":
        """Build a parking scenario from an artifact dictionary."""
        if data.get("type") != "parking":
            raise ValueError(f"Unsupported scenario type: {data.get('type')!r}")

        lot_data = data["parking_lot"]
        lot_config = {
            "rows": int(lot_data["rows"]),
            "cols": int(lot_data["cols"]),
            "layout": lot_data.get("layout", "single_side"),
            "origin_x": float(lot_data.get("origin_x", 0.0)),
            "origin_y": float(lot_data.get("origin_y", 0.0)),
            "slot_width": float(lot_data["slot_width"]),
            "slot_length": float(lot_data["slot_length"]),
            "gap_between_slots": float(lot_data["gap_between_slots"]),
            "aisle_width": float(lot_data["aisle_width"]),
            "slot_orientation_deg": float(
                lot_data.get("slot_orientation_deg", np.rad2deg(lot_data.get("slot_orientation_rad", 0.0)))
            ),
        }
        objects = tuple(ScenarioObject(**obj) for obj in data.get("objects", []))
        return cls(
            scenario_id=str(data["scenario_id"]),
            lot=ParkingLot(lot_config),
            goal_slot_index=int(data["goal_slot_index"]),
            parked_cars=objects,
        )


class ParkingScenarioFactory:
    """Create parking scenarios from configuration.

    This factory is the local replacement for the old simulator-owned scenario
    generation code. Future ADAS/OpenSCENARIO importers can construct the same
    `ParkingScenario` model without depending on `ParkingSimulationEnvironment`.
    """

    def __init__(
        self,
        parking_lot_config: dict[str, Any],
        occupancy: float = 0.5,
        seed: int = 42,
    ):
        self.parking_lot_config = parking_lot_config
        self.occupancy = occupancy
        self.seed = seed

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "ParkingScenarioFactory":
        scenario_config = config.get("scenario", {})
        return cls(
            parking_lot_config=config["parking_lot"],
            occupancy=float(scenario_config.get("occupancy", 0.5)),
            seed=int(scenario_config.get("seed", 42)),
        )

    def create(self, goal_slot_index: int = 0) -> ParkingScenario:
        lot = ParkingLot(self.parking_lot_config)
        rng = np.random.default_rng(self.seed)

        num_occupied = int(len(lot.slots) * self.occupancy)
        occupied_indices = rng.choice(len(lot.slots), num_occupied, replace=False)

        parked_cars: list[ScenarioObject] = []
        for idx in occupied_indices:
            slot_index = int(idx)
            if slot_index == goal_slot_index:
                continue

            slot = lot.get_slot(slot_index)
            parked_cars.append(
                ScenarioObject(
                    object_id=f"parked_car_{slot_index}",
                    object_type="parked_vehicle",
                    x=float(slot.x),
                    y=float(slot.y),
                    heading=float(slot.heading),
                    length=4.67,
                    width=1.86,
                    slot_index=slot_index,
                )
            )

        return ParkingScenario(
            scenario_id=f"parking_goal_{goal_slot_index}",
            lot=lot,
            goal_slot_index=goal_slot_index,
            parked_cars=tuple(parked_cars),
        )
