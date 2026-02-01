#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "io/scenario_loader.h"
#include "validators/speed_limit.h"
#include "validators/acceleration_limit.h"
#include "validators/lane_boundary.h"
#include "validators/collision_aabb.h"

static void PrintUsage()
{
    std::cout << "Usage:\n"
              << "  scenario_playback --traj data/sample_trajectory.csv --obs data/sample_obstacles.csv \\\n"
              << "    [--max_speed 15.0] [--max_accel 3.0] [--lane_half_width 1.8] [--ego_w 2.0] [--ego_h 4.5] \\\n"
              << "    [--out report.json]\n";
}

static bool GetArg(int argc, char **argv, const std::string &key, std::string &value)
{
    for (int i = 1; i + 1 < argc; ++i)
    {
        if (argv[i] == key)
        {
            value = argv[i + 1];
            return true;
        }
    }
    return false;
}

static double GetArgDouble(int argc, char **argv, const std::string &key, double defaultVal)
{
    std::string v;
    if (!GetArg(argc, argv, key, v))
        return defaultVal;
    return std::atof(v.c_str());
}

int main(int argc, char **argv)
{
    std::string trajPath, obsPath, outPath;
    if (!GetArg(argc, argv, "--traj", trajPath) || !GetArg(argc, argv, "--obs", obsPath))
    {
        PrintUsage();
        return 1;
    }
    GetArg(argc, argv, "--out", outPath);

    const double maxSpeed = GetArgDouble(argc, argv, "--max_speed", 15.0);
    const double maxAccel = GetArgDouble(argc, argv, "--max_accel", 3.0);
    const double laneHalf = GetArgDouble(argc, argv, "--lane_half_width", 1.8);
    const double egoW = GetArgDouble(argc, argv, "--ego_w", 2.0);
    const double egoH = GetArgDouble(argc, argv, "--ego_h", 4.5);

    asv::Scenario scenario;
    std::string err;
    if (!asv::ScenarioLoader::LoadFromCsv(trajPath, obsPath, scenario, err))
    {
        std::cerr << "Failed to load scenario: " << err << "\n";
        return 2;
    }

    asv::Report report;

    // Fill report metadata (makes output feel product-grade)
    report.meta.trajectoryFile = trajPath;
    report.meta.obstaclesFile = obsPath;
    report.meta.params.maxSpeedMps = maxSpeed;
    report.meta.params.maxAccelMps2 = maxAccel;
    report.meta.params.laneHalfWidthM = laneHalf;
    report.meta.params.egoBox.w = egoW;
    report.meta.params.egoBox.h = egoH;

    // Scenario time range (from trajectory)
    report.meta.scenarioStartT = scenario.ego.points.front().t;
    report.meta.scenarioEndT = scenario.ego.points.back().t;

    // Pluggable validators
    asv::SpeedLimitValidator vSpeed(maxSpeed);
    asv::AccelerationLimitValidator vAccel(maxAccel);
    asv::LaneBoundaryValidator vLane(laneHalf);
    asv::CollisionAabbValidator vColl(egoW, egoH);

    vSpeed.Validate(scenario, report);
    vAccel.Validate(scenario, report);
    vLane.Validate(scenario, report);
    vColl.Validate(scenario, report);

    const std::string json = report.ToJson();
    if (!outPath.empty())
    {
        std::ofstream ofs(outPath.c_str());
        ofs << json;
        std::cout << "Wrote report: " << outPath << "\n";
    }
    else
    {
        std::cout << json << "\n";
    }

    return 0;
}
