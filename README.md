# ParkingEnv

ParkingEnv is a monorepo for developing ADAS assisted-parking stacks against a
Python simulator.

The workspace has three top-level products:

- `simulator/`: Python simulator, scenario catalog support, scoring, rendering, and experiment runner
- `adas_stack/`: reusable C++ ADAS parking stack core
- `sim_bridge/`: C++ TCP/JSON adapter between the Python simulator and an ADAS stack

The bridge is intentionally outside the ADAS stack. The ADAS stack should stay
portable enough to later integrate into a vehicle architecture; the bridge is
only the local experiment adapter.

## Architecture

```text
simulator/
  scenario artifacts -> vehicle simulation -> scoring/rendering

TCP line-delimited JSON
  reset/state/result messages from simulator
  acceleration/steering commands from ADAS stack

adas_stack/
  planning + trajectory generation + tracking/control

sim_bridge/
  protocol and transport glue for local experiments
```

The simulator accepts only the vehicle-level actuator request:

- longitudinal acceleration in `m/s^2`
- front mid-axle steering angle in `rad`

`simulator.simulation.SimulatorController` clamps those commands to vehicle
limits before physics integration.

## Project Layout

- `simulator/simulation`: vehicle model, parking-lot geometry, scenarios, visualization, and scoring
- `simulator/experiment_runner.py`: headless catalog runner that talks to a C++ ADAS stack over TCP
- `simulator/adas_stack_catalog.py`: loads selectable ADAS stack executables
- `adas_stack/include` and `adas_stack/src`: C++ reference ADAS stack core
- `sim_bridge/include` and `sim_bridge/src`: TCP/JSON bridge executable
- `catalogs/adas_stacks.json`: dropdown-ready ADAS stack catalog for CLI and future UI
- `scripts/generate_parking_scenario.py`: scenario artifact generator
- `scripts/run_experiment.py`: script wrapper around the headless experiment runner
- `artifacts/scenarios`: generated scenario catalog artifacts

## C++ ADAS Stack Layout

```text
adas_stack
  include/parking_adas/domain       reusable ADAS data types
  include/parking_adas/common       math helpers
  include/parking_adas/planning     parking path and strategy interfaces
  include/parking_adas/control      low-speed control interfaces
  include/parking_adas/stack        ParkingAdasStack facade
  src                               stack implementation

sim_bridge
  include/sim_bridge/transport      TCP client
  include/sim_bridge/protocol       simulator JSON protocol
  src/app                           bridge executable entrypoint
```

`reference_parking_stack` is built as a library and does not know about TCP,
JSON, or the Python simulator.

## Build The C++ Stack

```bash
cmake -S . -B build
cmake --build build --config Release
```

The default ADAS catalog expects:

```text
build/Release/tcp_json_sim_bridge.exe
```

## Run Experiments

Run slot 0 with the catalog-selected C++ stack:

```bash
python -m simulator.experiment_runner --adas-stack reference_parking --slots 0
```

or:

```bash
python scripts/run_experiment.py --adas-stack reference_parking --slots 0
```

After editable install:

```bash
parking-experiment --adas-stack reference_parking --slots 0
```

Run the full default scenario catalog:

```bash
python -m simulator.experiment_runner --adas-stack reference_parking
```

List available ADAS stacks:

```bash
python -m simulator.experiment_runner --list-adas-stacks
```

Enable live visualization:

```bash
python -m simulator.experiment_runner --adas-stack reference_parking --slots 0 --render
```

Save final render images instead of opening a live window:

```bash
python -m simulator.experiment_runner --adas-stack reference_parking --save-render-dir artifacts/renders
```

The experiment runner launches the selected C++ stack by default. To start the
stack manually, run:

```bash
python -m simulator.experiment_runner --wait-for-external-stack --slots 0
```

Then in another terminal:

```powershell
.\build\Release\tcp_json_sim_bridge.exe --tcp-client --host 127.0.0.1 --port 8765
```

## Scenario Catalog

Generate or refresh the perpendicular scenario:

```bash
python scripts/generate_parking_scenario.py \
  --output artifacts/scenarios/single_side_empty_parking_lot.json \
  --scenario-id single_side_empty_perpendicular_parking_lot
```

Generate or refresh the parallel scenario:

```bash
python scripts/generate_parking_scenario.py \
  --output artifacts/scenarios/single_side_empty_parallel_parking_lot.json \
  --scenario-id single_side_empty_parallel_parking_lot \
  --slot-orientation-deg 0 \
  --slot-length 6.0
```

Generate obstacle variants:

```bash
python scripts/generate_parking_scenario.py \
  --output artifacts/scenarios/single_side_obstacle_parking_lot.json \
  --scenario-id single_side_obstacle_perpendicular_parking_lot \
  --occupancy 0.5 \
  --seed 7

python scripts/generate_parking_scenario.py \
  --output artifacts/scenarios/single_side_obstacle_parallel_parking_lot.json \
  --scenario-id single_side_obstacle_parallel_parking_lot \
  --slot-orientation-deg 0 \
  --slot-length 6.0 \
  --occupancy 0.5 \
  --seed 11
```

The default scenario catalog is:

- `artifacts/scenarios/single_side_empty_parking_lot.json`
- `artifacts/scenarios/single_side_empty_parallel_parking_lot.json`
- `artifacts/scenarios/single_side_obstacle_parking_lot.json`
- `artifacts/scenarios/single_side_obstacle_parallel_parking_lot.json`

For scenarios with parked cars, the runner tests only empty slots unless
`--slots` is passed explicitly.

## Future Web UI

The future UI should sit on top of the same Python services:

- scenario generation writes artifacts into `artifacts/scenarios`
- scenario catalog view reads `artifacts/scenarios`
- ADAS stack dropdown reads `catalogs/adas_stacks.json`
- experiment launch calls the same runner logic used by `parking-experiment`
- rendering can reuse `simulator.simulation.visualization`

This keeps headless full-catalog tests and UI-triggered experiments on the same
backend path.

## Development

```bash
pip install -e .
pytest -q
```

Recommended tools:

- `ruff` for linting
- `black` for formatting
- `pytest` for tests
