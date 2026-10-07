# Interview preparation

## A truthful 30-second project explanation

“I built a C++17 tool that replays recorded autonomy scenarios and checks speed,
acceleration, jerk, lane margins, oriented collisions and predictive TTC. I kept
the validation library independent of OpenCV, then added frame export and a small
classical image-processing lane demo. The engineering work includes strict input
handling, geometry tests, sanitizer checks and measured synthetic throughput.
The lane demo is validated on generated straight-line fixtures; I do not claim a
production perception system.”

## Three résumé bullets

- Developed a modular C++17 autonomy scenario validator with oriented bounding-box
  SAT collision checks, constant-velocity TTC, dynamic obstacles, vector
  acceleration/jerk metrics and deterministic JSON reporting.
- Implemented optional OpenCV bird's-eye playback and a classical lane-detection
  pipeline using Gaussian filtering, Canny edges, ROI masking, Hough segments and
  geometric filtering with synthetic ground-truth tests.
- Added GoogleTest/CTest unit and CLI integration coverage, warning-clean builds,
  ASan/UBSan verification, Linux CI configuration and reproducible benchmarks;
  validated 41 core checks and 54 OpenCV checks locally, with one codec-dependent
  video check skipped.

Use these only after reviewing and understanding the code. Include measured
throughput with its hardware, sample/obstacle counts and timed scope if asked.
Do not imply that GitHub CI has run until the workflow completes after publishing.

## Five likely technical questions and answers

### 1. Why use OBB/SAT instead of AABB collision?

AABB cannot represent heading and can report overlap for thin, rotated vehicles
that are separated. A rectangle's two local axes plus the other rectangle's two
axes are sufficient separating-axis candidates. I project centers and half
extents onto each axis; a single separated projection proves disjointness.
Equality counts as contact. The AABB class remains a reference, and tests compare
both methods at zero yaw and show a rotated AABB false positive. This method is
constant work per pair; a broad phase would be the next step for many obstacles.

### 2. How is TTC defined, and when is it misleading?

I compute first rectangle contact under constant translation and fixed yaw. On
each SAT axis, I solve an interval inequality for projected relative position and
velocity, then intersect the four intervals with nonnegative time. It can model
crossing obstacles; a center-distance/closing-speed formula could incorrectly
flag a near miss. Parallel separated motion has no TTC and existing contact has
TTC zero. The initial ego velocity is unknown, so only existing contact is known
at the first sample. Acceleration, steering and uncertainty invalidate the
constant-velocity prediction; it is not a proof about future trajectory samples.

### 3. Why are your derivatives more careful than simple speed differences?

Segment velocity measures displacement over an interval and belongs to its
midpoint. Acceleration differences vector velocities using the separation of
those midpoints; jerk uses the corresponding acceleration midpoint times.
Dividing by the newest interval is wrong for unequal spacing. Vector differences
also capture turning acceleration at constant speed. Tests use a quadratic path
with unequal intervals and a right-angle turn. These unsmoothed finite
differences amplify noise, so I expose assumptions and unavailable estimates
rather than inventing initial values.

### 4. Explain your vision pipeline and its limits.

Grayscale conversion is followed by Gaussian denoising and Canny edge detection.
A trapezoidal ROI removes irrelevant edges. Probabilistic Hough segments give
candidate markings; I reject horizontal, wrong-side and implausible candidates,
then fit x=a*y+b using segment-length-weighted endpoint regression. That form
avoids dividing by near-zero dx for vertical segments. Tests check bottom
coordinates against known synthetic fixtures, empty inputs, missing sides and
ROI behavior. Shadows, curves and arbitrary camera positions are not validated.
I export intermediate images to make failures inspectable. A world-coordinate
conversion would require calibrated intrinsics/extrinsics and ground-plane
assumptions, so the demo stays in pixels.

### 5. How did you verify performance and design for collaboration?

I used std::chrono::steady_clock around the full validation call, one warmup and
five repeats, recording min/median/max, workload size, validator count and a
checksum. Clear scenarios exercise rectangle distances; dense-contact scenarios
exercise report allocations. Generation, I/O, serialization and visualization
are outside the timer, and the workloads are not compared as an optimization
factor. The core is dependency-light; validators own only configuration and read
a shared context. unique_ptr makes plugin ownership explicit, optional values
encode unavailable measurements, and CI configurations cover GCC, Clang
sanitizers and OpenCV. Tests and mathematical documentation define contracts for
other contributors. Hosted job outcomes are recorded in VERIFICATION.md.

## Suggested code walkthrough

1. `include/core/validation.h` and `src/core/validation.cpp`: ownership, shared
   derivatives and extension interface.
2. `src/geometry/obb.cpp`: separating axes, clearance and swept intervals.
3. `src/vision/lane_detector.cpp`: explicit image stages and candidate regression.
4. `tests/core_tests.cpp` and `tests/vision_tests.cpp`: analytical and synthetic
   fixtures that could falsify the implementation.
5. `docs/PERFORMANCE.md`: timing scope, measured data and limits.
