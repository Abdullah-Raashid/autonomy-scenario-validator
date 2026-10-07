# Local verification record

Verified on 2026-10-07 (Asia/Tokyo), Apple M2 MacBook Air, 8 GB RAM,
macOS 27.0.1 arm64, Apple Clang 21.0.0 and CMake 4.1.1. The optional OpenCV
verification uses 4.12.0 built from source into a temporary prefix outside the
repository, with core/imgproc/imgcodecs/videoio and PNG support. It has no usable
MP4 writer. GoogleTest is pinned at 1.17.0.

## Results

| Check | Result |
|---|---|
| Original source baseline | Fresh C++14 build, smoke test and sample run succeeded before editing |
| Fresh source-only Release core | Configure with normal GoogleTest download, warnings-as-errors build, 40/40 CTest checks passed |
| Fresh source-only Release OpenCV | Warnings-as-errors build; 51 checks passed, 1 codec-dependent MP4 check skipped; 0 failures |
| Debug ASan/UBSan core | 40/40 checks passed, 0 sanitizer findings in the exercised code |
| Core without tests/OpenCV/network | `BUILD_TESTING=OFF` build and dynamic scenario CLI succeeded |
| clang-format 18.1.8 | CMake `format-check` target passed |
| Original sample + JSON | FAIL, 7 violations: 2 lane, 2 collision, 3 TTC; parsed as valid JSON |
| Dynamic sample + JSON | FAIL, 6 violations; parsed as valid JSON |
| Custom sample thresholds | FAIL, 11 violations; flags and output write succeeded |
| Headless playback | Six PNG frames for each sample scenario; images visually inspected |
| PNG roundtrip | Exported frame decoded pixel-identically to the rendered Mat |
| Synthetic lane pipeline | Both lanes detected; tolerance tests against known coordinates passed |
| PNG-input lane pipeline | Checked-in generated image processed; both lanes detected |
| MP4 request | Writer unavailable; CLI returned 2 with a clear diagnostic and PNGs intact |
| Benchmarks | Three actual Release workloads, each one warmup + five repeats; raw CSV retained |
| README sample JSON | Exact structural equality against a freshly generated current report |
| Repository whitespace | `git diff --check` passed |

CTest totals overlap: the OpenCV configuration includes the core checks. The
video skip is an explicit GoogleTest skip with a reason, not a passing codec test.
Synthetic lane tolerances are 12 pixels at known bottom coordinates; passing them
does not establish real-road perception accuracy. Tests make no cross-platform
bit-identical image claim.

The source-only copies contained source, tests, configuration and intentional
assets, without `.git`, compiled binaries or build caches. CMake configured fresh
build directories under `/tmp`. The OpenCV build reused the downloaded GoogleTest
source and a temporary OpenCV prefix. README commands were exercised with the
corresponding executable/build paths; no local system package installation was
performed. Benchmark timing scope and raw values are in [PERFORMANCE.md](PERFORMANCE.md).

## Reproduction

Use the README commands in a fresh clone. The core path uses no OpenCV;
`ASV_WITH_OPENCV=ON` adds the optional tools and tests. If dependencies are already
available, `OpenCV_DIR` can point at your installation's CMake package and
`FETCHCONTENT_SOURCE_DIR_GOOGLETEST` at GoogleTest 1.17.0 source.

No developer-specific paths, cache files or build outputs are required.

## Publication verification

The final polish is being verified from committed, freshly cloned source.
Post-commit checks and exact hosted workflow outcomes will be recorded here after
completion. clang-tidy remains optional and has not been run locally. The local
MP4 writer is unavailable; PNG output and the explicit codec-dependent skip remain
verified.
