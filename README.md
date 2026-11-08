# ADAS Environment

ADAS Environment is a monorepo for developing and comparing assisted-parking
ADAS stacks against a Python simulator.

The workspace has four top-level products:

- `simulator/`: Python simulator, scenario catalog support, scoring, rendering, and experiment runner
- `motion_api/`: shared vehicle-level motion contract
- `reference_adas_stacks/`: composable C++ stack core and selectable reference implementations
- `sim_bridge/`: C++ TCP/JSON adapter between the Python simulator and an ADAS stack

The bridge is intentionally outside the ADAS stack. The ADAS stack should stay
portable enough to later integrate into a vehicle architecture; the bridge is
only the local experiment adapter.

For contributor-oriented architecture documentation, see the
[architecture guide index](docs/README.md).

## Architecture

```text
simulator/
  scenario artifacts -> vehicle simulation -> scoring/rendering

TCP line-delimited JSON
  reset/state/result messages from simulator
  acceleration/steering commands from ADAS stack

motion_api/
  vehicle-level state and command contract

reference_adas_stacks/
  shared planning/tracking interfaces and stack orchestration
  hybrid_astar and reeds_shepp_local planner implementations
  pure_pursuit tracker implementation

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
- `motion_api/include`: shared vehicle-level motion types
- `reference_adas_stacks/core`: reusable C++ interfaces, orchestration, and common components
- `reference_adas_stacks/include` and `reference_adas_stacks/src`: selectable planner implementations
- `sim_bridge/include` and `sim_bridge/src`: TCP/JSON bridge executable
- `catalogs/adas_stacks.json`: dropdown-ready ADAS stack catalog for CLI and future UI
- `scripts/generate_parking_scenario.py`: scenario artifact generator
- `scripts/run_experiment.py`: script wrapper around the headless experiment runner
- `artifacts/scenarios`: generated scenario catalog artifacts

## C++ ADAS Stack Layout

```text
motion_api
  include                            shared vehicle-level contract

reference_adas_stacks
  core/include and core/src          shared ADAS pipeline and stack facade
  include and src                     replaceable planner implementations
  core/src/control                    reference tracker implementation

sim_bridge
  include/sim_bridge/transport      TCP client
  include/sim_bridge/protocol       simulator JSON protocol
  src/app                           bridge executable entrypoint
```

The C++ stack core is built as a library and does not know about TCP, JSON, or
the Python simulator. The catalog selects a planner implementation while the
stack composes it with the configured tracker and shared pipeline stages.

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
python -m simulator.experiment_runner --adas-stack hybrid_astar --slots 0
```

or:

```bash
python scripts/run_experiment.py --adas-stack hybrid_astar --slots 0
```

After editable install:

```bash
adas-experiment --adas-stack hybrid_astar --slots 0
```

Run the default smoke catalog:

```bash
python -m simulator.experiment_runner --adas-stack hybrid_astar
```

List available ADAS stacks:

```bash
python -m simulator.experiment_runner --list-adas-stacks
```

Compare the reference path planners under the same simulator and tracker:

```bash
python -m simulator.experiment_runner --adas-stack hybrid_astar --scenario-suite smoke
python -m simulator.experiment_runner --adas-stack reeds_shepp_local --scenario-suite smoke
```

Enable live visualization:

```bash
python -m simulator.experiment_runner --adas-stack hybrid_astar --slots 0 --render
```

Save final render images instead of opening a live window:

```bash
python -m simulator.experiment_runner --adas-stack hybrid_astar --save-render-dir artifacts/renders
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

Scenario artifacts are split into two deterministic suites:

- `artifacts/scenarios/smoke`: four quick regression cases: empty perpendicular,
  empty parallel, and parking between two cars in both layouts.
- `artifacts/scenarios/benchmark`: static-obstacle, tight-gap, offset-vehicle,
  end-slot, 60-degree angled, and bollard cases.

Run the smoke suite, which is the default:

```bash
python -m simulator.experiment_runner --adas-stack hybrid_astar --scenario-suite smoke
```

Run the broader planner-development benchmark:

```bash
python -m simulator.experiment_runner --adas-stack hybrid_astar --scenario-suite benchmark
```

Run every built-in artifact:

```bash
python -m simulator.experiment_runner --adas-stack hybrid_astar --scenario-suite all
```

Each artifact runs its declared `goal_slot_index` by default. To evaluate every
unoccupied slot in each selected layout instead, use `--all-empty-slots`.

The core parallel challenge is:

- `artifacts/scenarios/smoke/parallel_between_two_cars.json`

Its target is slot 3, with parked cars in slots 2 and 4. The 6.0 m bay leaves
0.665 m longitudinal clearance at each end around the 4.67 m reference vehicle.

Regenerate the checked-in smoke and benchmark catalogs with:

```bash
python scripts/generate_scenario_catalog.py
```

Generate a new baseline artifact with:

```bash
python scripts/generate_parking_scenario.py \
  --output artifacts/scenarios/benchmark/my_scenario.json \
  --scenario-id my_scenario
```

Use explicit artifacts during development without changing a suite:

```bash
python -m simulator.experiment_runner \
  --adas-stack hybrid_astar \
  --scenario-artifact artifacts/scenarios/benchmark/my_scenario.json
```

## Future Web UI

The future UI should sit on top of the same Python services:

- scenario generation writes artifacts into `artifacts/scenarios`
- scenario catalog view reads `artifacts/scenarios`
- ADAS stack dropdown reads `catalogs/adas_stacks.json`
- experiment launch calls the same runner logic used by `adas-experiment`
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
