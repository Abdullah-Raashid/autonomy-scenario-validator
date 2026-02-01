#pragma once
#include <vector>
#include <string>
#include "core/types.h"

namespace asv
{

    struct Obstacle
    {
        // static obstacle for MVP
        std::string id;
        AABB2 box;
    };

    struct Obstacles
    {
        std::vector<Obstacle> items;
    };

} // namespace asv
