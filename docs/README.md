# Architecture Documentation

The monorepo contains three independently replaceable applications. Each guide
describes one boundary and links to the others where they meet.

- [ADAS stack](ADAS_STACK_ARCHITECTURE.md): reusable C++ parking behavior.
- [Planner and tracker contributor guide](PLANNER_TRACKER_CONTRIBUTOR_GUIDE.md):
  implement and compare a motion-planning idea through the public C++ interfaces.
- [Simulation bridge](SIM_BRIDGE_ARCHITECTURE.md): local C++ TCP/JSON adapter.
- [Simulator](SIMULATOR_ARCHITECTURE.md): Python scenarios, physics, scoring,
  experiments, and rendering.
- [ADAS parking benchmark](ADAS_PARKING_BENCHMARK.md): reproducible planner
  comparisons, metrics, and output artifacts.
