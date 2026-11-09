# Planner And Tracker Contributor Guide

This guide is the entry point for an ADAS contributor. You can develop a new
motion-planning idea without changing the Python simulator, TCP bridge, JSON
protocol, or parking-stack orchestration.

## Extension Boundary

Only these public headers are required:

| Goal | Interface | Input | Output |
| --- | --- | --- | --- |
| Create geometric motion | `planning/path_planner.hpp` `PathPlanner` | `PlanRequest` | `vector<Waypoint>` |
| Follow a trajectory | `control/trajectory_tracker.hpp` `TrajectoryTracker` | trajectory and `VehicleState` | `MotionCommand`, `TrackingStatus` |
| Assign speeds and gear stops | `planning/trajectory_generator.hpp` `TrajectoryGenerator` | path and vehicle limits | trajectory |
| Constrain an existing trajectory | `planning/trajectory_optimizer.hpp` `TrajectoryOptimizer` | trajectory and request | trajectory |

`Waypoint` is the shared motion object. It carries rear-axle pose, direction,
curvature, target speed, and an explicit gear-change marker. The planner emits
it, the generator and optimizer preserve its geometry, the tracker consumes it,
and the simulator visualizes it.

## Add A Path Planner

1. Add the planner implementation under `reference_adas_stacks/include/` and
   derive from the shared `PathPlanner` interface in
   `motion_api/include/parking_adas/planning/path_planner.hpp`.
2. Implement `plan(const PlanRequest&)`. Construct one `PlanningContext` for
   the request and use its collision checker and goal evaluator. The context
   owns the consistent request snapshot and immutable `CollisionScene`.
   Connectors that need all three receive the context rather than independently
   assembled request, checker, and goal evaluator objects.
3. Override `should_replan(const TrackingStatus&, const PlanRequest&)` only if
   your planner needs a strategy-specific recovery policy.
4. Register a stable ID in `reference_adas_stacks/core/src/planning/path_planner.cpp`.
5. Add a catalog entry in `catalogs/adas_stacks.json` with
   `--path-planner my_planner`.

```cpp
class MyPlanner final : public parking_adas::PathPlanner {
public:
    std::vector<parking_adas::Waypoint> plan(
        const parking_adas::PlanRequest& request) override;
    const parking_adas::PlanningDiagnostics& diagnostics() const override;
    const char* name() const override { return "my_planner"; }
};
```

## Add A Tracker

1. Add the tracker interface implementation under
   `reference_adas_stacks/core/include/` and derive from `TrajectoryTracker`.
2. `reset()` receives the exact trajectory generated from the selected path.
3. `command()` returns only longitudinal acceleration and front mid-axle
   steering angle. Do not modify the planned trajectory.
4. `assess()` reports tracking observations; it must not decide to replan.
5. Register the tracker ID in
   `reference_adas_stacks/core/src/control/trajectory_tracker.cpp`, then select
   it with `--trajectory-tracker my_tracker` in a catalog entry.

Use `control/pure_pursuit_tracker.*` as the concrete reference, but depend only
on `control/trajectory_tracker.hpp` from shared stack code.

```cpp
class MyTracker final : public parking_adas::TrajectoryTracker {
public:
    void reset(const std::vector<parking_adas::Waypoint>& trajectory,
               parking_adas::VehicleParameters vehicle) override;
    parking_adas::MotionCommand command(
        const parking_adas::VehicleState& state) override;
    parking_adas::TrackingStatus assess(
        const parking_adas::VehicleState& state) override;
    const char* name() const override { return "my_tracker"; }
};
```

## Run And Test

```powershell
cmake --build build --config Release
python -m simulator.experiment_runner --list-adas-stacks
python -m simulator.experiment_runner --adas-stack hybrid_astar --scenario-suite smoke
ctest --test-dir build -C Release --output-on-failure
pytest -q
```

Start with an explicit smoke artifact while iterating. Use `--render` only for
inspection; catalog regression runs should remain headless.

## Rules Of The Road

- The Python simulator is authoritative for vehicle dynamics and parking
  success. Keep C++ bicycle-model behavior aligned with its parity tests.
- A successful manoeuvre requires every vehicle corner inside the selected slot.
- Keep algorithm-specific logic inside the planner or tracker implementation.
  `ParkingAdasStack` only composes pipeline stages and applies the actuator
  safety check.
- Keep TCP, JSON, scenario parsing, and rendering out of an ADAS implementation.
