# Measured validation throughput

Measured 2026-10-07 (Asia/Tokyo) on a **MacBook Air, Apple M2, 8 GB RAM**, running
**macOS 27.0.1, arm64**, with **Apple Clang 21.0.0 (clang-2100.3.34.2)**,
**CMake 4.1.1**, C++17 and CMake **Release** compilation (`-O3 -DNDEBUG`).
The benchmark binary comes from a fresh clone of committed source (`8a531c3`),
measured after all verification builds completed. OpenCV is not
linked into the benchmark. No artificial result or comparison to the original
project is included.

| Workload | Samples | Obstacles | Median | Throughput | Violations/run |
|---|---:|---:|---:|---:|---:|
| clear | 10,000 | 20 | 34.19 ms | 292,482 samples/s | 0 |
| clear | 100,000 | 20 | 345.70 ms | 289,269 samples/s | 0 |
| stress | 10,000 | 8 | 8.17 ms | 1,224,715 samples/s | 160,000 |

Each measurement uses six validators, one untimed warmup and five timed engine
runs. Report count and sample count contribute to the printed checksum. The
minimum, median, maximum and raw throughput are preserved in
[benchmark-results.csv](benchmark-results.csv). Results are observations on this
machine and vary with workload, background activity, compiler and hardware.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 4
./build/asv_benchmark --samples 10000 --obstacles 20 --repeats 5
./build/asv_benchmark --samples 100000 --obstacles 20 --repeats 5
./build/asv_benchmark --samples 10000 --obstacles 8 --repeats 5 --stress
```

Default scenarios have ego and obstacles translating along +x at 10 m/s; obstacles
are laterally separated by 8 m, remain within an invariant relative layout, and
are rotated by 0.1 rad. They exercise exact clearance for disjoint OBBs without
violation allocations. Stress scenarios use eight overlapping moving OBBs and
produce dense collision/TTC reports. They short-circuit distance calculation on
overlap and have fewer obstacles; their higher throughput must not be interpreted
as a speedup for the same workload.

The timer surrounds `ValidationEngine::Validate`, including invariant checking,
shared kinematics preparation, all validators, report construction and violation
allocations. It ends when the report has been constructed. Scenario generation,
CSV reads, report serialization, final report destruction, rendering and codec
work are excluded. The benchmark is single-threaded. It does not measure a camera
pipeline, deadlines, p99 latency or real-time behavior.

Performance decisions are modest and inspectable: kinematics are shared between
rules, source/context buffers reserve capacity, obstacles are read by reference,
and geometry uses fixed-size corner/axis arrays. No claimed optimization factor
is presented without an equivalent baseline measurement. Runtime remains O(N*M)
for obstacles; clearance work on every disjoint pair and memory for O(K)
violations are the natural next profiling targets. A broad phase or streaming
report should be added only when a concrete workload justifies its complexity.
