#pragma once
#include "model/trajectory.h"
#include "model/obstacles.h"

namespace asv
{

    struct Scenario
    {
        Trajectory ego;
        Obstacles obstacles;
    };

} // namespace asv
