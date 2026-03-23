# KarlsPlayground

Realistic parking simulation focused on low-speed maneuvering with planning + control modules.

## Project Layout

- `KarlsPlayground/simulation`: vehicle model, environment, scenario, and geometry utilities
- `KarlsPlayground/control`: controllers, path planner, and trajectory tracker
- `KarlsPlayground/visualization`: matplotlib-based rendering
- `KarlsPlayground/demo_runner.py`: reusable demo workflow
- `demo.py`: thin local entrypoint script

## Quick Start

1. Create and activate a virtual environment.
2. Install dependencies:

```bash
pip install -r requirements.txt
```

3. Run demo:

```bash
python demo.py
```

or using package entrypoint:

```bash
parking-demo
```

## Development

The project uses `pyproject.toml` for packaging metadata and tool configuration.

Recommended tools:

- `ruff` for linting
- `black` for formatting
- `pytest` for tests
