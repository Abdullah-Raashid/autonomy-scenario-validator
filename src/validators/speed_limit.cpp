#include "validators/speed_limit.h"
#include <algorithm>
#include <locale>
#include <sstream>
namespace asv {
void SpeedLimitValidator::Evaluate(const ValidationContext &ctx, Report &r) const {
    for (std::size_t i = 0; i < ctx.frames.size(); ++i) {
        const auto &f = ctx.frames[i];
        if (!f.velocity)
            continue;
        const double value = Norm(*f.velocity);
        r.metrics.maxSpeedMps = std::max(r.metrics.maxSpeedMps.value_or(0.0), value);
        if (value > threshold_) {
            std::ostringstream text;
            text.imbue(std::locale::classic());
            text << "velocity magnitude=" << value << " exceeds limit=" << threshold_;
            r.Add(f.t, Name(), Severity::Warning, text.str(), i);
        }
    }
}
} // namespace asv
