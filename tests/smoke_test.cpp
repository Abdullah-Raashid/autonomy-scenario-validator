#include <cassert>
#include <string>
#include "io/scenario_loader.h"
#include "validators/speed_limit.h"
#include "core/report.h"

int main()
{
    asv::Scenario s;
    std::string err;
    bool ok = asv::ScenarioLoader::LoadFromCsv("data/sample_trajectory.csv", "data/sample_obstacles.csv", s, err);
    assert(ok);

    asv::Report r;
    asv::SpeedLimitValidator v(1.0); // very low to force violations
    v.Validate(s, r);
    assert(!r.violations.empty());
    return 0;
}
