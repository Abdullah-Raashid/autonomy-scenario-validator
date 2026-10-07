#pragma once
#include "core/types.h"
#include <array>
#include <optional>

namespace asv {
struct OBB2 {
    Vec2 center;
    double length = 4.5, width = 2.0, yaw = 0.0;
};
// Geometry inputs require finite centers/yaw and positive finite dimensions.
std::array<Vec2, 4> Corners(const OBB2 &box);
bool Intersects(const OBB2 &a, const OBB2 &b);
double BoxDistance(const OBB2 &a, const OBB2 &b);
// Exact first contact for translating rectangles with fixed yaw and constant velocity.
std::optional<double> TimeToCollision(const OBB2 &a, Vec2 va, const OBB2 &b, Vec2 vb);
double LaneMargin(const OBB2 &ego, double halfWidth);
} // namespace asv
