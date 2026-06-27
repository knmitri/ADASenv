# KarlsPlayground

Realistic parking simulation focused on low-speed maneuvering with planning + control modules.

## Project Layout

- `KarlsPlayground/scenarios`: scenario data models and parking scenario factories
- `KarlsPlayground/simulation`: vehicle model, environment, parking lot geometry, and simulation utilities
- `KarlsPlayground/control`: controllers, path planner, and trajectory tracker
- `KarlsPlayground/visualization`: matplotlib-based rendering
- `KarlsPlayground/demo_runner.py`: reusable demo workflow
- `scripts`: manual diagnostics and plotting utilities

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
