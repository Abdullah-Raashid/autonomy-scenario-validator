# Contributing

Use C++17, keep the validator core independent of OpenCV, and describe coordinate
and numerical assumptions for algorithm changes. Build out of source. Changes
should explain the concrete problem and observable result, with focused tests
for meaningful boundary behavior. Keep generated outputs out of commits except
intentional small documentation assets with their generation commands.

Before proposing changes:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DASV_WARNINGS_AS_ERRORS=ON
cmake --build build --parallel 4
ctest --test-dir build --output-on-failure
```

The formatting convention is clang-format **18**:

```sh
clang-format -i $(git ls-files '*.cpp' '*.h')
cmake --build build --target format-check
```

`format-check` is available when `clang-format` is on PATH at configure time.
`.clang-tidy` contains optional bug-prone, performance and selected modernization
checks; enable compiler-time analysis with `-DASV_ENABLE_CLANG_TIDY=ON` when
clang-tidy is installed. Static analysis is advisory and is not a mandatory CI
gate. The CI matrix runs GCC Release, Clang ASan/UBSan Debug and OpenCV Release,
plus clang-format 18. CI execution is only verified by a completed GitHub run.

Public APIs are in `include/`. Derive from `asv::IValidator`, implement a const
`Evaluate` method, and register owned validators with `ValidationEngine`. Rules
should avoid recomputing kinematics, preserve deterministic output and report
unavailable measurements honestly. Detailed contracts are in
[ALGORITHMS.md](docs/ALGORITHMS.md).
