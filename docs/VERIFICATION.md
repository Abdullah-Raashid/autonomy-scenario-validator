# Verification record

Verified on 2026-10-07 (Asia/Tokyo), Apple M2 MacBook Air, 8 GB RAM,
macOS 27.0.1 arm64, Apple Clang 21.0.0 and CMake 4.1.1. The optional OpenCV
verification uses 4.12.0 built from source into a temporary prefix outside the
repository, with core/imgproc/imgcodecs/videoio and PNG support. It has no usable
MP4 writer. GoogleTest is pinned at 1.17.0.

## Final committed-source checks

The five implementation/documentation commits were completed before verification.
Source commit `8a531c3a4968ad5fa38f8cf2a5b3ce8b8573b23f` was cloned with
`git clone --no-local` into a new temporary directory. All builds below used that
clone and fresh build directories; no untracked source or previous build cache
was copied. Later verification-record edits change documentation only.

| Check | Actual result |
|---|---|
| Release core | Normal pinned GoogleTest download; warnings-as-errors configure/build succeeded; 41/41 CTest checks passed |
| Release OpenCV | Warnings-as-errors configure/build succeeded; 54 passed, 1 codec-dependent MP4 check skipped, 0 failures |
| Debug core ASan/UBSan | Warnings-as-errors configure/build succeeded; 41/41 checks passed; no sanitizer findings in the exercised code |
| Formatting | clang-format 18.1.8 CMake `format-check` target passed |
| README sample report | Valid JSON; FAIL with 7 violations: 2 collision, 2 lane, 3 TTC; exact structural equality with `docs/sample-report.json` |
| README playback command | Six PNG frames generated; contact/lane-departure frame visually inspected |
| README image-input vision command | Both lanes detected; all stage images and combined panel exported and visually inspected |
| Synthetic vision command | Both lanes detected from the generated fixture; repeatability/coordinate-tolerance tests passed |
| Release benchmark | All three README workloads ran after builds completed; one warmup and five timed runs each; raw CSV recorded |
| Commit buildability | Core feature commit `dc70b97` and vision feature commit `6d8c75d` independently configured and built with tests off and warnings as errors |
| Repository hygiene | Clean tree before verification; no tracked build directories; deliberate PNG assets tracked; build/binary/report/IDE outputs ignored; no developer-specific absolute paths in the current tracked tree |

CTest totals overlap: the OpenCV configuration includes the core checks. The
video skip is an explicit GoogleTest skip with a reason, not a passing codec test.
Synthetic lane tolerances are 12 pixels at known bottom coordinates; passing them
does not establish real-road perception accuracy. Tests make no cross-platform
bit-identical image claim.

## Earlier checks retained during the upgrade

The original C++14 baseline built and ran before editing. Additional checks
covered a core build with `BUILD_TESTING=OFF`, dynamic-scenario JSON and playback,
custom CLI thresholds, deterministic JSON ordering, PNG pixel round trips, and
the unavailable MP4 writer's explicit diagnostic/exit code with PNGs intact.
The final suite adds cross-row CSV diagnostic and extreme-viewport renderer
regressions. See the named CTest cases rather than treating these observations
as extra independent tests.

## Reproduction

Use the README commands from the repository root in a fresh clone. The core path
uses no OpenCV; `ASV_WITH_OPENCV=ON` adds the optional tools and tests. For the
local OpenCV and sanitizer checks, existing GoogleTest 1.17.0 source was supplied
through `FETCHCONTENT_SOURCE_DIR_GOOGLETEST`; OpenCV was found through
`OpenCV_DIR`. These are dependency overrides, not required source files.
No developer-specific paths, cache files or build outputs are required.

The benchmark timing scope, environment and latest raw values are in
[PERFORMANCE.md](PERFORMANCE.md) and [benchmark-results.csv](benchmark-results.csv).
The three medians are 34.19 ms (10,000 samples/20 obstacles), 345.70 ms
(100,000/20), and 8.17 ms (10,000/8 overlapping obstacles). These observations
do not establish real-time performance.

## Publication verification

Published to `main`. [Hosted C++ CI run 37613380983](https://github.com/Abdullah-Raashid/autonomy-scenario-validator/actions/runs/37613380983)
completed successfully for commit `594b234e43e4b8e00ce8525a3e0173139e9fe7e1`
on 2026-10-07. All four Ubuntu 24.04 jobs passed:

| Hosted job | Actual result |
|---|---|
| GCC Release | GCC 13.3; warnings-as-errors build; 41/41 checks; sample JSON and benchmark smoke passed |
| Clang sanitizers | Clang 18.1.3; Debug ASan/UBSan with leak detection; 41/41 checks; sample and benchmark smoke passed |
| OpenCV Release | System OpenCV 4.6.0; 55/55 checks passed with no skip, including MP4 encode/decode; six dynamic-scenario PNGs and both detected lanes exported |
| Formatting | clang-format 18 check passed |

The OpenCV artifact `opencv-demo-output` was uploaded successfully (2,917,178
bytes; artifact ID `11479410937`). The earlier
[run 37613079468](https://github.com/Abdullah-Raashid/autonomy-scenario-validator/actions/runs/37613079468)
also passed all four jobs, but reported Node 20 action-runtime deprecation
warnings. Checkout and artifact upload were updated to documented Node 24
versions, and the rerun passed without those warnings.

The published GitHub README was inspected in a browser: both PNGs and all five
badges loaded, the Mermaid diagram rendered, and the hero, commands, JSON and
tables displayed correctly. Published assets match fresh demo outputs byte for
byte. Relative documentation links resolve; the current tracked tree contains
no build outputs or developer-specific absolute paths. Original repository
history is preserved; removing generated files from the current tree does not
erase them from earlier commits.

Optional clang-tidy has not been run locally. PNG export is verified on macOS and
Linux; MP4 export is verified in hosted Linux CI and remains unavailable in the
local minimal OpenCV installation.
