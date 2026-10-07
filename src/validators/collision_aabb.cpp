#include "validators/collision_aabb.h"
namespace asv {
void CollisionAabbValidator::Evaluate(const ValidationContext &ctx, Report &r) const {
    for (std::size_t i = 0; i < ctx.frames.size(); ++i) {
        const auto &f = ctx.frames[i];
        const AABB2 ego{f.position.x, f.position.y, xExtent_, yExtent_};
        for (const auto &o : ctx.scenario.obstacles.items) {
            const auto pose = ObstacleAt(o, f.t - ctx.frames.front().t);
            const AABB2 box{pose.center.x, pose.center.y, o.box.w, o.box.h};
            if (Intersects(ego, box))
                r.Add(f.t, Name(), Severity::Error, "AABB contact: " + o.id, i);
        }
    }
}
} // namespace asv
