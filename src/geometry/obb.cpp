#include "geometry/obb.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace asv {
namespace {
std::array<Vec2, 2> Axes(const OBB2 &b) {
    return {{{std::cos(b.yaw), std::sin(b.yaw)}, {-std::sin(b.yaw), std::cos(b.yaw)}}};
}
double Radius(const OBB2 &b, Vec2 axis) {
    const auto local = Axes(b);
    return 0.5 *
           (b.length * std::abs(Dot(local[0], axis)) + b.width * std::abs(Dot(local[1], axis)));
}
std::array<Vec2, 4> TestAxes(const OBB2 &a, const OBB2 &b) {
    const auto aa = Axes(a), ba = Axes(b);
    return {aa[0], aa[1], ba[0], ba[1]};
}
double PointSegmentDistance(Vec2 p, Vec2 a, Vec2 b) {
    const auto d = b - a;
    const double squared = Dot(d, d);
    if (squared == 0.0)
        return Norm(p - a); // edge collapsed at floating-point precision
    if (!std::isfinite(squared))
        throw std::invalid_argument("geometry exceeds numeric range");
    const double u = std::clamp(Dot(p - a, d) / squared, 0.0, 1.0);
    return Norm(p - (a + d * u));
}
} // namespace
std::array<Vec2, 4> Corners(const OBB2 &b) {
    const auto axes = Axes(b);
    const auto x = axes[0] * (b.length * 0.5), y = axes[1] * (b.width * 0.5);
    return {b.center + x + y, b.center - x + y, b.center - x - y, b.center + x - y};
}
bool Intersects(const OBB2 &a, const OBB2 &b) {
    for (auto axis : TestAxes(a, b))
        if (std::abs(Dot(b.center - a.center, axis)) > Radius(a, axis) + Radius(b, axis))
            return false;
    return true; // touching is collision
}
double BoxDistance(const OBB2 &a, const OBB2 &b) {
    if (Intersects(a, b))
        return 0.0;
    const auto ac = Corners(a), bc = Corners(b);
    double distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 4; ++j) {
            distance = std::min(distance, PointSegmentDistance(ac[i], bc[j], bc[(j + 1) % 4]));
            distance = std::min(distance, PointSegmentDistance(bc[i], ac[j], ac[(j + 1) % 4]));
        }
    if (!std::isfinite(distance))
        throw std::invalid_argument("geometry exceeds numeric range");
    return distance;
}
std::optional<double> TimeToCollision(const OBB2 &a, Vec2 va, const OBB2 &b, Vec2 vb) {
    double enter = 0.0, leave = std::numeric_limits<double>::infinity();
    for (auto axis : TestAxes(a, b)) {
        const double d = Dot(b.center - a.center, axis), v = Dot(vb - va, axis);
        const double r = Radius(a, axis) + Radius(b, axis);
        if (!std::isfinite(d) || !std::isfinite(v) || !std::isfinite(r))
            throw std::invalid_argument("TTC exceeds numeric range");
        if (v == 0.0) {
            if (std::abs(d) > r)
                return std::nullopt;
            continue;
        }
        double t0 = (-r - d) / v, t1 = (r - d) / v;
        if (t0 > t1)
            std::swap(t0, t1);
        enter = std::max(enter, t0);
        leave = std::min(leave, t1);
        if (enter > leave)
            return std::nullopt;
    }
    if (!std::isfinite(enter))
        return std::nullopt;
    return enter;
}
double LaneMargin(const OBB2 &b, double halfWidth) {
    return halfWidth - std::abs(b.center.y) - Radius(b, {0.0, 1.0});
}
} // namespace asv
