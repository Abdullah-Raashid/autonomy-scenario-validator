#pragma once
#include "model/obstacles.h"
#include "model/trajectory.h"

namespace asv {

struct Scenario {
    Trajectory ego;
    Obstacles obstacles;
};

} // namespace asv
