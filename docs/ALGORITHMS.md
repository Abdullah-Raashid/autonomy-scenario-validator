# Data model and validation mathematics

## Units and coordinates

World positions use meters, time seconds, yaw radians counterclockwise from +x.
World +x is forward along the straight lane; +y is left. Ego `length` is along
its local forward axis and `width` along its local lateral axis. Legacy obstacle
`w` is length along local x and `h` is width along local y; at zero yaw these
remain the original world-axis extents. Ego defaults are length 4.5 m, width 2 m.
The old CLI interpreted those axes ambiguously; schema v2 makes them explicit.

The lane is the fixed corridor `-halfWidth <= y <= halfWidth`. This project does
not model a curved road, road curvature, map coordinates or camera calibration.

Supported CSV headers (column order is exact):

```csv
t,x,y
t,x,y,yaw
id,cx,cy,w,h
id,cx,cy,w,h,yaw,vx,vy
```

Headers are mandatory. Fields can be quoted, including commas and doubled quote
characters. CRLF, a UTF-8 BOM and blank lines are supported; multiline quoted
fields are rejected. Numbers must be finite complete decimal tokens; scientific
notation is accepted. Trajectory CSV requires at least two samples, nonnegative
strictly increasing timestamps, and exact column counts. Obstacles require
unique nonempty IDs and positive dimensions. A header-only obstacle CSV is valid;
omitting `--obs` means no obstacles. A provided missing file is an error.

Loading is transactional: a failure leaves the caller's scenario unchanged.
`CsvReader` diagnostics use physical line numbers; the scenario loader reports
logical CSV row numbers after blank lines are removed; cross-row derived failures
are labeled scenario consistency errors. Library validation also
checks programmatically created data. An empty trajectory is rejected; a single
library sample can be used for geometric checks with unavailable derivatives.

## Shared kinematics

For sample `i`, displacement velocity is

```
v_i = (p_i - p_(i-1)) / (t_i - t_(i-1))
tv_i = t_(i-1) + (t_i - t_(i-1))/2
```

Vector acceleration differences successive segment velocities at their midpoint
times, rather than dividing by the most recent sample interval:

```
a_i = (v_i - v_(i-1)) / (tv_i - tv_(i-1))
ta_i = tv_(i-1) + (tv_i - tv_(i-1))/2
j_i = (a_i - a_(i-1)) / (ta_i - ta_(i-1))
```

Acceleration and jerk limits use Euclidean vector magnitudes. A change in driving
direction therefore contributes acceleration even at constant scalar speed.
Derivative values are diagnostic estimates, associated with the ending sample
index; their mathematical timestamps are the midpoints above. There is no
smoothing: noisy positions or tiny sample intervals amplify numerical derivatives.

Speed needs two samples; acceleration three; jerk four. Missing estimates are
`std::nullopt` internally and JSON `null`. Overflow/nonfinite derived values are
rejected. Heading comes from explicit yaw, otherwise from backward displacement;
at a stop it retains the previous heading. The initial heading follows the first segment when it moves, otherwise zero. Initial velocity
remains unknown even though a forward segment can estimate initial orientation.

## Oriented rectangle collision

An OBB has center `c`, unit forward axis `u=(cos(yaw), sin(yaw))`, lateral axis
`v=(-sin(yaw), cos(yaw))`, length `L` and width `W`. Projection radius on axis `n`:

```
r(n) = (L/2)*abs(dot(u,n)) + (W/2)*abs(dot(v,n))
```

The separating axis theorem checks the two local axes of each rectangle. If any
axis has `abs(dot(c_b-c_a,n)) > r_a(n)+r_b(n)`, the boxes are disjoint; otherwise
they overlap or touch. Contact counts as collision. All checks use double
precision, without an artificial tolerance or safety inflation.

For disjoint rectangles, minimum Euclidean clearance is the minimum of vertex-to-
edge distances in both directions. Point-to-segment projection is clamped to
`[0,1]`. Overlap/contact clearance is zero. Tiny edges that collapse at floating
point precision are treated as points; out-of-range geometry is rejected.
`near_collision` warns for positive clearance strictly below `nearDistance`.
`CollisionAabbValidator` remains a world-axis baseline and is not in the default
engine; it deliberately ignores yaw, including obstacle yaw.

These collision checks apply at the input samples. A fast vehicle can pass through
an obstacle between samples. TTC is a prediction and does not make the sampled
collision rule a complete continuous collision guarantee.

## Dynamic obstacles and TTC

Each obstacle's CSV pose is its pose at the trajectory's **first timestamp**:

```
c_obstacle(t) = c_initial + velocity * (t - t_start)
```

Yaw and dimensions stay fixed. Ego velocity is the latest measured segment
velocity. TTC predicts first contact if both OBBs continue translating at constant
velocities while holding their current orientations.

For each SAT axis, with `d=dot(c_b-c_a,n)`, `q=dot(v_b-v_a,n)` and combined radius
`r`, solve `-r <= d+q*t <= r`. Each axis gives an interval. Intersect all four
intervals with `[0,+infinity)`; the entry time is the TTC. A stationary separated
axis or disjoint intervals means there is no predicted contact (`null`), even if
center distance is decreasing. Existing contact gives zero. At the initial sample,
only existing contact is reported because measured ego velocity is unavailable.
TTC warns strictly below the configured threshold. Contact can produce both an
error from collision and a zero-TTC warning; these describe separate diagnostics.

This swept SAT model is exact for constant translation with fixed orientation;
it does not extrapolate acceleration, steering, uncertainty or actual future
trajectory samples. It is useful for reasoning about the model, not a safety
certificate. An unbounded prediction horizon is used for the aggregate minimum;
the violation threshold selects short-horizon warnings.

## Lane margin

The lateral extent of the entire ego body is

```
extent_y = (L/2)*abs(sin(yaw)) + (W/2)*abs(cos(yaw))
margin = halfWidth - abs(center_y) - extent_y
```

A negative margin is departure. Equality is within the lane. The report keeps the
minimum margin, so narrow clearances and orientation effects are inspectable.

## Classical vision lane demo

The optional pipeline accepts 8-bit BGR or grayscale images. It uses explicit
stages: BGR-to-gray conversion, 5x5 Gaussian filtering (sigma 1.3), Canny with L2
gradient, a trapezoidal ROI, probabilistic Hough segments, then geometric filtering.
The ROI top defaults to 55% of image height; Hough length/gap parameters scale
with image height. Candidate slopes use `x = a*y+b`, avoiding division by `dx`
for nearly vertical lines. Horizontal marks, out-of-range slopes, wrong-side
segments and implausible bottom intercepts are rejected. Each side is fitted by
segment-length-weighted least squares on endpoint pixels; crossing fits are
rejected. Missing sides stay unavailable. All intermediate stages are exported.

Synthetic fixtures have known straight lane coordinates, deterministic mild noise
and distractors. Tests check tolerance against that ground truth, blank images,
one missing side, ROI behavior and repeated processing. They do not measure
accuracy on real roads. No pixel coordinates are converted into world meters:
that requires camera intrinsics/extrinsics and a defensible ground-plane mapping.
Curves, shadows, severe occlusion, lane changes and arbitrary camera views are
outside this demonstration's validation coverage.

OpenCV references: [Canny and HoughLinesP](https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html),
[Gaussian filtering](https://docs.opencv.org/4.x/d4/d86/group__imgproc__filter.html).

## Reporting, complexity and ownership

The engine owns polymorphic validators with `unique_ptr`; validators read one
borrowed scenario and prepared kinematics context. Its six default validators
are speed, acceleration, jerk, lane, collision/clearance and TTC. Add a rule by
implementing `IValidator::Evaluate` and transferring it to `AddValidator`.
Geometry, CSV I/O and OpenCV are separate layers. The context and renderer must
not outlive the scenario/report they borrow. Validator order and obstacle order
make report output stable; `std::map` makes count serialization deterministic.
JSON strings preserve valid UTF-8, escape control characters and reject malformed
UTF-8. Numeric output uses the classic locale and 17-digit precision. Schema v2
adds typed severity, sample indices, explicit dimensions and nullable metrics.

For N samples, M obstacles and V scalar rules, runtime is O(N*(V+M)); geometric
checks use constant-size arrays without heap allocations per pair. Prepared
kinematics uses O(N) memory and the report uses O(K) for K violations; dense
violations can dominate allocations. The benchmark includes those allocations.
No broad-phase spatial index, streaming report or parallel execution is claimed.
Rendering is separate, with a fixed viewport and an invertible metric-to-pixel
transform: x right, y up. Grid spacing adapts to the viewport and its tick loop is
bounded; unrepresentable spans/pixel coordinates are rejected. Video uses one frame per input sample at a chosen fixed
FPS, so irregular scenario timing is not preserved or interpolated.
