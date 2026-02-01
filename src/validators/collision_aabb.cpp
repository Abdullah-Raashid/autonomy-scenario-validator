#include "validators/collision_aabb.h"
#include <sstream>

namespace asv
{

    void CollisionAabbValidator::Validate(const Scenario &scenario, Report &report) const
    {
        const std::vector<TrajectoryPoint> &pts = scenario.ego.points;

        for (size_t i = 0; i < pts.size(); ++i)
        {
            AABB2 ego;
            ego.cx = pts[i].x;
            ego.cy = pts[i].y;
            ego.w = egoW_;
            ego.h = egoH_;

            for (size_t j = 0; j < scenario.obstacles.items.size(); ++j)
            {
                const Obstacle &o = scenario.obstacles.items[j];
                if (Intersects(ego, o.box))
                {
                    std::ostringstream oss;
                    oss << "ego intersects obstacle id=" << o.id;
                    report.Add(pts[i].t, Name(), "error", oss.str());
                }
            }
        }
    }

} // namespace asv
