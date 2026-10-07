#include "validators/ttc.h"
#include <algorithm>
namespace asv {
void TtcValidator::Evaluate(const ValidationContext &ctx, Report &r) const {
    for (std::size_t i = 0; i < ctx.frames.size(); ++i) {
        const auto &f = ctx.frames[i];
        const auto ego = EgoAt(f, ego_);
        for (const auto &o : ctx.scenario.obstacles.items) {
            const auto box = ObstacleAt(o, f.t - ctx.frames.front().t);
            // First sample has no measured velocity; only existing contact is known.
            if (!f.velocity && !Intersects(ego, box))
                continue;
            const auto ttc = TimeToCollision(ego, f.velocity.value_or(Vec2{}), box, o.velocity);
            if (!ttc)
                continue;
            r.metrics.minTtcS = std::min(r.metrics.minTtcS.value_or(*ttc), *ttc);
            if (*ttc < threshold_)
                r.Add(f.t, Name(), Severity::Warning,
                      "predicted contact within TTC threshold: " + o.id, i);
        }
    }
}
} // namespace asv
