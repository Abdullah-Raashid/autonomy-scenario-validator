#include "validators/collision_obb.h"
#include <algorithm>
namespace asv {
void CollisionObbValidator::Evaluate(const ValidationContext &ctx, Report &r) const {
    for (std::size_t i = 0; i < ctx.frames.size(); ++i) {
        const auto &f = ctx.frames[i];
        const auto ego = EgoAt(f, ego_);
        for (const auto &o : ctx.scenario.obstacles.items) {
            const double distance = BoxDistance(ego, ObstacleAt(o, f.t - ctx.frames.front().t));
            r.metrics.minObstacleDistanceM =
                std::min(r.metrics.minObstacleDistanceM.value_or(distance), distance);
            if (distance == 0.0)
                r.Add(f.t, Name(), Severity::Error, "contact with obstacle " + o.id, i);
            else if (distance < nearDistance_)
                r.Add(f.t, "near_collision", Severity::Warning,
                      "clearance below threshold: " + o.id, i);
        }
    }
}
} // namespace asv
