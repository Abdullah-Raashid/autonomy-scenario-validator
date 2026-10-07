#include "validators/jerk_limit.h"
#include <algorithm>
#include <locale>
#include <sstream>
namespace asv {
void JerkLimitValidator::Evaluate(const ValidationContext &ctx, Report &r) const {
    for (std::size_t i = 0; i < ctx.frames.size(); ++i) {
        const auto &f = ctx.frames[i];
        if (!f.jerk)
            continue;
        const double value = Norm(*f.jerk);
        r.metrics.maxJerkMps3 = std::max(r.metrics.maxJerkMps3.value_or(0.0), value);
        if (value > threshold_) {
            std::ostringstream text;
            text.imbue(std::locale::classic());
            text << "jerk magnitude=" << value << " exceeds limit=" << threshold_;
            r.Add(f.t, Name(), Severity::Warning, text.str(), i);
        }
    }
}
} // namespace asv
