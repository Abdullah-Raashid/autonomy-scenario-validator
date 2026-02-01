#pragma once
#include <string>
#include <vector>
#include <cstdint>

namespace asv {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

struct AABB2 {
  // axis-aligned bounding box centered at (cx, cy)
  double cx = 0.0;
  double cy = 0.0;
  double w  = 0.0;  // width
  double h  = 0.0;  // height
};

inline bool Intersects(const AABB2& a, const AABB2& b) {
  const double ax0 = a.cx - a.w * 0.5, ax1 = a.cx + a.w * 0.5;
  const double ay0 = a.cy - a.h * 0.5, ay1 = a.cy + a.h * 0.5;
  const double bx0 = b.cx - b.w * 0.5, bx1 = b.cx + b.w * 0.5;
  const double by0 = b.cy - b.h * 0.5, by1 = b.cy + b.h * 0.5;

  const bool overlapX = (ax0 <= bx1) && (ax1 >= bx0);
  const bool overlapY = (ay0 <= by1) && (ay1 >= by0);
  return overlapX && overlapY;
}

} // namespace asv
