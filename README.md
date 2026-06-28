# KarlsPlayground

Realistic parking simulation focused on low-speed maneuvering and parking motion-planner challenges.

## Project Layout

- `KarlsPlayground/simulation`: vehicle model, environment, parking lot geometry, command limits, and scoring
- `KarlsPlayground/simulation/scenarios`: scenario data models, parking scenario factories, and artifact I/O
- `KarlsPlayground/simulation/visualization`: matplotlib-based simulator rendering
- `KarlsPlayground/planning`: planner interface, baseline planner, path planning, trajectory generation, and tracking
- `KarlsPlayground/demo_runner.py`: reusable demo workflow
- `scripts`: scenario generation and challenge runners

## Planner Interface

A parking planner is expected to plan from a scenario and then return simulator commands:

```python
command = planner.compute_motion_command(state, goal_heading, dynamic_obstacles=[])
```

The simulator accepts only:

- longitudinal acceleration in `m/s^2`
- front steering angle in `rad`

`KarlsPlayground.simulation.SimulatorController` clamps those commands to vehicle limits before
calling `ParkingSimulationEnvironment.step(acceleration, steering_angle)`.

## Quick Start

1. Create and activate a virtual environment.
2. Install dependencies:

```bash
pip install -r requirements.txt
```

3. Run demo:

Generate a reusable scenario artifact once:

```bash
python scripts/generate_parking_scenario.py
```

Then run the demo against that stored scenario:

```bash
python -m KarlsPlayground.demo_runner
```

or using package entrypoint:

```bash
parking-demo
```

Run the six-slot parking challenge headlessly:

```bash
python scripts/run_parking_challenge.py
```

or using the package entrypoint:

```bash
parking-challenge
```

Use a custom planner without touching the simulator:

```bash
python scripts/run_parking_challenge.py --planner your_module:YourPlanner
```

The planner class should use the same constructor shape as `BaselineParkingPlanner`
and implement the `KarlsPlayground.planning.ParkingPlanner` protocol.

By default, the first scenario is stored at
`artifacts/scenarios/single_side_empty_parking_lot.json`. This local
artifact directory is intentionally separated from the simulator so it can later
be replaced by an external artifact store such as JFrog Artifactory, S3, or an
ADAS scenario database.

## Development

The project uses `pyproject.toml` for packaging metadata and tool configuration.

Recommended tools:

- `ruff` for linting
- `black` for formatting
- `pytest` for tests
