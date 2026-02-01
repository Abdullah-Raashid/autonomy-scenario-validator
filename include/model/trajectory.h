#pragma once
#include <vector>
#include <string>
#include "core/types.h"

namespace asv
{

    struct TrajectoryPoint
    {
        double t = 0.0; // seconds
        double x = 0.0; // meters
        double y = 0.0; // meters
    };

    struct Trajectory
    {
        std::vector<TrajectoryPoint> points;
    };

} // namespace asv
