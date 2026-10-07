#include "validators/lane_boundary.h"
#include <algorithm>
namespace asv {
void LaneBoundaryValidator::Evaluate(const ValidationContext &ctx, Report &r) const {
    for (std::size_t i = 0; i < ctx.frames.size(); ++i) {
        const auto &f = ctx.frames[i];
        const double margin = LaneMargin(EgoAt(f, ego_), halfWidth_);
        r.metrics.minLaneMarginM = std::min(r.metrics.minLaneMarginM.value_or(margin), margin);
        if (margin < 0.0)
            r.Add(f.t, Name(), Severity::Error, "vehicle footprint crosses lane boundary", i);
    }
}
} // namespace asv
