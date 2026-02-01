#include "validators/lane_boundary.h"
#include <sstream>
#include <cmath>

namespace asv
{

    void LaneBoundaryValidator::Validate(const Scenario &scenario, Report &report) const
    {
        const std::vector<TrajectoryPoint> &pts = scenario.ego.points;
        for (size_t i = 0; i < pts.size(); ++i)
        {
            const double y = pts[i].y;
            if (std::fabs(y) > halfWidth_)
            {
                std::ostringstream oss;
                oss << "y=" << y << " outside lane corridor [-" << halfWidth_ << ", +" << halfWidth_ << "]";
                report.Add(pts[i].t, Name(), "error", oss.str());
            }
        }
    }

} // namespace asv
