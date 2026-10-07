#pragma once
#include "core/types.h"
#include <string>
#include <vector>

namespace asv {
struct Obstacle {
    std::string id;
    AABB2 box; // w is longitudinal length, h lateral width at yaw=0
    double yaw = 0.0;
    Vec2 velocity; // constant world velocity in m/s
};
struct Obstacles {
    std::vector<Obstacle> items;
};
} // namespace asv
