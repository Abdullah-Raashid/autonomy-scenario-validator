# Autonomy Scenario Playback & Validation Tool

A lightweight C++ tool for replaying recorded vehicle trajectories and validating common autonomy constraints. The tool loads a driving scenario from simple CSV inputs, executes modular validation checks over time, and outputs a structured JSON report suitable for both downstream tooling.

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

## Project Structure

```
include/ # Public headers
core/ # Common types and reporting
io/ # CSV parsing and scenario loading
model/ # Scenario and trajectory data models
validators/ # Individual validation rules

src/ # Implementations
apps/ # Executable entry point
data/ # Sample trajectory and obstacle data
tests/ # Minimal smoke test
```

Validators are implemented independently and operate on a shared scenario model.

## Build

### Requirements
- CMake ≥ 3.12
- C++14-compatible compiler

### Build steps

```
bash
mkdir build
cd build
cmake ..
cmake --build . -j
```

Usage

Run validation and print the report to stdout:

```./scenario_playback \
  --traj ../data/sample_trajectory.csv \
  --obs ../data/sample_obstacles.csv \
  --out report.json
```

Optional parameters
```
--max_speed        
--max_accel        
--lane_half_width  
--ego_w            Ego vehicle bounding box width (m)
--ego_h            Ego vehicle bounding box height (m)
```

---

## Output

The tool generates a structured JSON report containing:

- Scenario inputs and validation parameters
- Time range of the scenario
- Summary with overall result of either PASS, WARN, or FAIL
- Counts by severity and validation rule
- Detailed per-violation information with timestamps
