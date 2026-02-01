#include "validators/acceleration_limit.h"
#include <cmath>
#include <sstream>

namespace asv
{

    static double Speed(const TrajectoryPoint &a, const TrajectoryPoint &b)
    {
        const double dt = b.t - a.t;
        if (dt <= 1e-9)
            return 0.0;
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        return std::sqrt(dx * dx + dy * dy) / dt;
    }

    void AccelerationLimitValidator::Validate(const Scenario &scenario, Report &report) const
    {
        const std::vector<TrajectoryPoint> &pts = scenario.ego.points;
        if (pts.size() < 3)
            return;

        for (size_t i = 2; i < pts.size(); ++i)
        {
            const double v0 = Speed(pts[i - 2], pts[i - 1]);
            const double v1 = Speed(pts[i - 1], pts[i]);
            const double dt = pts[i].t - pts[i - 1].t;
            if (dt <= 1e-9)
                continue;

            const double a = (v1 - v0) / dt;
            if (std::fabs(a) > maxAccelMps2_)
            {
                std::ostringstream oss;
                oss << "accel=" << a << " m/s^2 exceeds max=" << maxAccelMps2_ << " m/s^2";
                report.Add(pts[i].t, Name(), "warn", oss.str());
            }
        }
    }

} // namespace asv
