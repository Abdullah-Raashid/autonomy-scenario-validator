# Autonomy Scenario Playback & Validation Tool

A lightweight C++ tool for replaying recorded vehicle trajectories and validating common autonomy constraints. The tool loads a driving scenario from simple CSV inputs, executes modular validation checks over time, and outputs a structured JSON report suitable for both human inspection and downstream tooling.

This project is designed to reflect real-world autonomy validation workflows, emphasizing modularity, reproducibility, and clear reporting.

---

## Features

- Replay ego vehicle trajectories over time
- Validate common autonomy constraints:
  - Speed limit
  - Acceleration limit
  - Lane boundary adherence
  - Collision checks (axis-aligned bounding boxes)
- Pluggable validator architecture for easy extension
- Reproducible, product-style JSON reports with:
  - Scenario metadata and parameters
  - Summary statistics and overall pass/fail result
  - Detailed per-timestep violations
- Simple command-line interface
