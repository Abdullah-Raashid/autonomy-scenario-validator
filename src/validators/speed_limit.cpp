#include "validators/speed_limit.h"
#include <cmath>
#include <sstream>

namespace asv
{

    void SpeedLimitValidator::Validate(const Scenario &scenario, Report &report) const
    {
        const std::vector<TrajectoryPoint> &pts = scenario.ego.points;
        for (size_t i = 1; i < pts.size(); ++i)
        {
            const double dt = pts[i].t - pts[i - 1].t;
            if (dt <= 1e-9)
                continue;
            const double dx = pts[i].x - pts[i - 1].x;
            const double dy = pts[i].y - pts[i - 1].y;
            const double v = std::sqrt(dx * dx + dy * dy) / dt;

            if (v > maxSpeedMps_)
            {
                std::ostringstream oss;
                oss << "speed=" << v << " m/s exceeds max=" << maxSpeedMps_ << " m/s";
                report.Add(pts[i].t, Name(), "warn", oss.str());
            }
        }
    }

} // namespace asv
