# ADAS Parking Benchmark

The benchmark compares `hybrid_astar` and `reeds_shepp_local` using the same
scenario artifacts, vehicle and simulation configuration, and
`pure_pursuit` tracker in closed-loop mode. Planner-only mode directly invokes
the C++ planner with the same serialized planning request. Neither mode changes
planner settings.

## Build and run

Build the simulator bridge in Release mode. For Linux with a
single-configuration generator:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target tcp_json_sim_bridge -j
```

For Windows with Visual Studio (multi-configuration):

```powershell
cmake -S . -B build
cmake --build build --config Release --target tcp_json_sim_bridge
```

`_release_executable()` checks the `Release` subdirectory for multi-config
generators and the build root for single-config builds; it rejects a
single-configuration build whose `CMAKE_BUILD_TYPE` is not `Release`.

Run the seven-artifact benchmark suite with five measured runs per
planner/scenario and one warm-up per planner/scenario:

```powershell
python -m simulator.benchmark --scenario-suite benchmark
```

Run one scenario, for example:

```powershell
python -m simulator.benchmark `
  --scenario-artifact artifacts\scenarios\benchmark\parallel_end_slot.json `
  --repetitions 5 --warmup-count 1
```

The default `--mode accelerated-closed-loop` runs the full stack and simulator,
with rendering disabled. The loop advances discrete simulation steps as fast as
the machine permits. Inspection found no sleep or real-time/frame pacing in the
simulator step or experiment loop, so this mode makes the existing non-rendered
behavior explicit rather than changing the simulation. The configured `dt`,
vehicle dynamics, controller cadence, actuator limits, and termination checks
are unchanged.

For planner-only measurements, bypassing TCP, tracking, and vehicle execution:

```powershell
python -m simulator.benchmark `
  --mode planner-only `
  --scenario-artifact artifacts\scenarios\benchmark\parallel_end_slot.json `
  --repetitions 5 --warmup-count 1
```

Planner-only mode feeds the C++ planner the same serialized reset request
fields used in the closed-loop protocol. `PlanningDiagnostics.planning_time_ms`
is the planner latency; `planner_only_process_wall_seconds` includes process
startup and request/response overhead. Closed-loop results use the separate
`closed_loop_initial_planning_latency_ms` field. Planner-only trials report
path length, waypoints, direction changes, and discrete path validity, but do
not report episode duration or parking execution success.

Change the measured repetition and warm-up counts with `--repetitions` and
`--warmup-count`. Warm-ups are retained in raw results but excluded from CSV
aggregates. Use `--max-steps` to apply the same diagnostic step cap to both
planners, and `--plots` to add comparison plots. Rendering is disabled during
benchmark runs. Every run creates a unique directory beneath
`artifacts\benchmarks` unless `--output-root` is specified; existing runs are
not overwritten.

The bridge executable must exist in the Release configuration under the
selected `--build-dir`. Debug executables are rejected so their timing is not
mistaken for Release performance.

## Outputs

Each run contains:

- `metadata.json`: commit, OS/architecture, compiler, Python, build, scenario
  hashes/IDs, tracker, vehicle/simulation settings, timeouts, repetitions, and
  the comparison-input fingerprint; SHA-256, size and modification time for
  the actual bridge executable; and relevant CMake behavior/build options.
  The executable hash, Git commit and worktree state are reported separately:
  the benchmark cannot establish that an existing build was produced from the
  current source revision.
- `trials\*.json` and `raw_trials.json`: full individual trial results,
  including warm-ups and captured failure reasons.
- `individual_trials.csv`: measured trials only.
- `scenario_aggregates.csv` and `planner_aggregates.csv`: trial-count-weighted
  effectiveness rates and available latency/path/episode statistics.
- `benchmark_report.md`: concise comparison and metric limitations.
- `logs\*.log`: runner output per trial.
- Optional mode-specific `valid_plan_rate.png` (planner-only) or
  `parking_success_rate.png` (closed-loop), plus `planning_latency.png` and
  `path_length.png`.

## Interpreting metrics

- **Initial planning latency** is the C++ planner's
  `PlanningDiagnostics.planning_time_ms` from the initial ready response. It
  excludes subprocess startup, TCP connection, handshake, and Python-side
  simulation. It does not include trajectory generation/optimization unless
  the planner includes those operations in its diagnostic.
- **Simulated duration** is `simulation_steps * dt`. **Episode wall duration**
  is measured independently from reset through simulator termination; it
  includes planning, TCP exchanges, replanning, and simulation. Their ratio is
  the simulation speed factor. Full trial wall time additionally includes
  process startup, connection/handshake, and shutdown. The runner also records
  the average duration of each Python `sim.step()` call.
- **Path length** is the Euclidean sum over actual initial-plan coordinates.
  Waypoint count is reported separately. Direction changes are missing when
  the planner does not expose direction data.
- **Path validity/collision** checks reported waypoints with the simulator's
  vehicle geometry and collision rules and checks the endpoint against the
  goal slot and heading tolerance. It is discrete waypoint validation, not a
  continuous swept-path guarantee. Initial-plan and final-commanded-path
  validity are reported separately.
- **Success, collision, stuck, timeout, and failure rates** use trials with a
  recorded outcome for that metric; aggregate CSVs include corresponding
  observation counts. Failed/timed-out trials remain in outputs. A missing
  measurement is left blank and does not enter that metric's statistic.
  Aggregate success is calculated from counts, not from unweighted scenario
  percentages.
- Latencies are sensitive to host load and thermal state. Compare planners on
  the same machine and build, and use repeated runs; the benchmark does not
  claim an improvement without measured results.

The built-in scenarios are deterministic and disable random spawning. Planner
order alternates across trials to reduce ordering bias. Both planners receive
the same artifact contents and vehicle/simulation settings; closed-loop runs
also use the same configured tracker. Closed-loop planning latency reports the
initial plan only and does not expose per-replan latency.

## Optional C++ planner integration tests

The regular Python suite excludes tests marked `integration`. After building
the Release bridge as above, run the direct planner checks with:

```powershell
python -m pytest -m integration tests\integration\test_planner_only.py
```

```bash
python -m pytest -m integration tests/integration/test_planner_only.py
```

The tests invoke both planner IDs with the deterministic `parallel_end_slot`
request, validate the response schema and planning diagnostics, parse the path,
and validate it against simulator geometry. They skip with a reason if the
expected Release executable is unavailable and fail explicitly if the planner
process exceeds the timeout. The standard unit command remains
`python -m pytest -q`.
