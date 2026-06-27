"""KarlsPlayground - A realistic Audi A5 parking simulation package."""

__version__ = "1.0.0"

from .control import ParkingController
from .scenarios import ParkingScenario, ParkingScenarioFactory, ScenarioObject
from .simulation import AudiA5, ParkingLot, ParkingSimulationEnvironment, ParkingSlot, Vehicle
from .visualization import SimulationVisualizer

__all__ = [
    "ParkingSimulationEnvironment",
    "AudiA5",
    "Vehicle",
    "ParkingLot",
    "ParkingSlot",
    "ParkingController",
    "ParkingScenario",
    "ParkingScenarioFactory",
    "ScenarioObject",
    "run_demo",
    "SimulationVisualizer",
]


def __getattr__(name):
    if name == "run_demo":
        from .demo_runner import run_demo

        return run_demo
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
