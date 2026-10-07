#pragma once
#include <cmath>

namespace asv {
struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};
inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline double Dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double Norm(Vec2 a) { return std::hypot(a.x, a.y); }

// Baseline: w spans world x, h spans world y (independent of vehicle heading).
struct AABB2 {
    double cx = 0.0, cy = 0.0, w = 0.0, h = 0.0;
};
inline bool Intersects(const AABB2 &a, const AABB2 &b) {
    return std::abs(a.cx - b.cx) <= (a.w + b.w) * 0.5 && std::abs(a.cy - b.cy) <= (a.h + b.h) * 0.5;
}
} // namespace asv
