#pragma once
#include <optional>
#include <vector>

namespace asv {
struct TrajectoryPoint {
    double t = 0.0;            // seconds, nonnegative and strictly increasing
    double x = 0.0, y = 0.0;   // meters, world x forward, y left
    std::optional<double> yaw; // radians CCW; inferred from displacement if omitted
};
struct Trajectory {
    std::vector<TrajectoryPoint> points;
};
} // namespace asv
