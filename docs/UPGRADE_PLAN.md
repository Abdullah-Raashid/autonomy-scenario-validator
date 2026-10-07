# Upgrade plan and baseline audit

The starting point is commit `58b8248`: C++14, one library, a CSV CLI, four
validators and one assertion-based smoke test. All source, headers, samples,
CMake and tracked-file inventory were inspected before editing. A fresh baseline
build and sample run succeeded; the sample reported two lane departures.

## Findings

- `atof` accepts malformed input as zero; malformed rows are silently skipped.
- CSV headers, finite values, dimensions and timestamp ordering are unchecked.
- Derivatives silently ignore invalid time intervals; acceleration uses scalar
  speed and the wrong time interval for unequal sample spacing.
- AABB collision ignores yaw; the ego dimensions are ambiguous.
- Lane validation checks only the vehicle center.
- Report escaping misses control characters, serialization depends on locale,
  and report file writes are unchecked.
- No CTest registration, README, CI, visualizations or performance evidence.
- Generated `build/` files and binaries are committed.

## Execution plan

1. Establish C++17 target-based builds, strict parsing and explicit invariants.
2. Prepare shared kinematics; implement OBB SAT, exact rectangle clearance,
   constant-velocity swept SAT TTC, jerk and full-body lane margins.
3. Add GoogleTest/CTest coverage of numerical, geometry, I/O and CLI boundaries.
4. Add an optional OpenCV library and separate headless playback/vision tools.
   Generate synthetic lane images and inspect exported frames.
5. Add a deterministic synthetic benchmark, measured results, compiler warnings,
   format configuration and Linux CI including sanitizers and OpenCV.
6. Document reproducible commands, mathematics, limitations, architecture,
   interview preparation and a documented commit sequence.
7. Verify fresh core and OpenCV builds, Release/Debug, sanitizers, reports,
   demos and benchmarks; remove only generated tracked artifacts.

No production ADAS, calibrated camera-to-world projection, real-time guarantees,
or road-image accuracy will be claimed. Core builds must work without OpenCV.

## Completed outcome

The plan was executed: C++17 target-based core and optional OpenCV targets; strict
CSV/CLI and schema-v2 reports; shared derivatives, OBB contact/clearance, dynamic
obstacles and swept SAT TTC; GoogleTest and CLI integration suites; generated
visual demos; actual benchmarks; CI/format/sanitizer configuration; and technical
documentation. Generated tracked build files are removed from the repository.

Final committed-source core checks: 41 passed. OpenCV checks: 54 passed, one MP4
writer check skipped. ASan/UBSan core checks: 41 passed. PNG demos and both lane
input modes were exercised. The local OpenCV build lacks an MP4 encoder. Hosted
Linux CI passed all four jobs, including all 55 OpenCV-enabled checks and MP4
encode/decode. Optional clang-tidy has not been run locally. See [VERIFICATION.md](VERIFICATION.md)
for the complete evidence and [INTERVIEW_PREP.md](INTERVIEW_PREP.md) for résumé/interview material.
